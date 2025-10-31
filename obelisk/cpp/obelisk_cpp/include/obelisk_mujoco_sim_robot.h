#ifndef OBELISK_MUJOCO_SIM_ROBOT_H
#define OBELISK_MUJOCO_SIM_ROBOT_H

#ifdef OBELISK_USE_MUJOCO

#pragma once

#include <cmath>
#include <cstdarg>
#include <filesystem>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/grid_cells.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "obelisk_sensor_msgs/msg/obk_joint_encoders.hpp"
#include "obelisk_sim_robot.h"

namespace obelisk {
    template <typename ControlMessageT> class ObeliskMujocoRobot : public ObeliskSimRobot<ControlMessageT> {
      public:
        explicit ObeliskMujocoRobot(const std::string& name)
            : ObeliskSimRobot<ControlMessageT>(name), viz_key_("viz_geom_pub") {
            this->template declare_parameter<std::string>("mujoco_setting", "");
            this->template declare_parameter<std::string>(
                "ic_keyframe",
                "ic"); // Parameter is the name of a mujoco key frame used as the initial condition. Defualts to "ic".

            mujoco_sim_instance_    = this;
            activation_complete_    = false;
            configuration_complete_ = false;
            mujoco_setup_           = false;
        }

        ~ObeliskMujocoRobot() { mujoco_sim_instance_ = nullptr; }

        /**
         * @brief Configures the node
         *
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskSimRobot<ControlMessageT>::on_configure(prev_state);

            // Read in the config string
            std::string mujoco_setting = this->get_parameter("mujoco_setting").as_string();
            auto mujoco_config_map     = this->ParseConfigStr(mujoco_setting);

            // Get config params
            xml_path_             = GetXMLPath(mujoco_config_map);      // Required
            std::string robot_pkg = GetRobotPackage(mujoco_config_map); // Optional
            // Search for the model
            if (!std::filesystem::exists(xml_path_)) {
                if (robot_pkg != "None" && robot_pkg != "none") {
                    std::string share_directory = ament_index_cpp::get_package_share_directory(robot_pkg);
                    xml_path_                   = "mujoco" / xml_path_;
                    xml_path_                   = share_directory / xml_path_;
                } else {
                    RCLCPP_ERROR_STREAM(this->get_logger(),
                                        "Provided Mujoco XML is NOT an absolute path and robot_pkg is None or not "
                                        "specified. Please provide a valid Mujoco XML path.");
                }
            }

            RCLCPP_INFO_STREAM(this->get_logger(), "XML Path:" << xml_path_);

            num_steps_per_viz_ = GetNumStepsPerViz(mujoco_config_map);

            configuration_complete_ = true;
            
            this->declare_parameter("height_map_geom_group", std::vector<long>{0});
            this->declare_parameter("height_map_grid_size", std::vector<double>{0.});
            this->declare_parameter("height_map_grid_spacing", 0.0);
            height_map_grid_size_ = this->get_parameter("height_map_grid_size").as_double_array();
            height_map_grid_spacing_ = this->get_parameter("height_map_grid_spacing").as_double();
            height_map_geom_group_ = this->get_parameter("height_map_geom_group").as_integer_array();

            ParseSensorString(mujoco_config_map.at("sensor_settings"));

            try {
                ParseVizString(mujoco_config_map.at("viz_geoms"));
            } catch (const std::exception& e) {
                RCLCPP_INFO_STREAM(this->get_logger(), "No geoms to visualize in the simulator.");
            }

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief activates up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskSimRobot<ControlMessageT>::on_activate(prev_state);

            activation_complete_ = true;

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief deactivates up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskSimRobot<ControlMessageT>::on_deactivate(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief cleans up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskSimRobot<ControlMessageT>::on_cleanup(prev_state);
            activation_complete_    = false;
            configuration_complete_ = false;

            // Reset data
            nu_          = -1;
            num_sensors_ = 0;

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief shutsdown the node the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskSimRobot<ControlMessageT>::on_shutdown(prev_state);
            activation_complete_    = false;
            configuration_complete_ = false;

            // Reset data
            nu_          = -1;
            num_sensors_ = 0;

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Apply the control message.
         * We assume that the control message is a vector of control inputs and is fully compatible with the data.ctrl
         * field of a mujoco model. YOU MUST CHECK THIS YOURSELF!
         */
        void ApplyControl(const ControlMessageT& msg) override { SetSharedData(msg.u_mujoco); }

        // -----------------------------------------//
        // ----------- Simulation Entry ----------- //
        // ---------------------------------------- //
        /**
         * @brief Configures the visulaizer and enters the Mujoco simulation loop.
         */
        void RunSimulator() override {
            while (!configuration_complete_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            RCLCPP_INFO_STREAM(this->get_logger(), "Setting up Mujoco visualization.");

            // load the mujoco model
            char error[1000] = "Could not load binary model";
            model_           = mj_loadXML(xml_path_.c_str(), 0, error, 1000);

            time_step_ = model_->opt.timestep;
            RCLCPP_INFO_STREAM(this->get_logger(), "Mujoco model loaded with " << time_step_ << " timestep.");

            nu_ = model_->nu;
            RCLCPP_INFO_STREAM(this->get_logger(), "Mujoco model loaded with " << nu_ << " inputs.");
            shared_data_.resize(nu_);

            if (!model_) {
                throw std::runtime_error("Could not load Mujoco model from the XML!");
            }

            data_ = mj_makeData(model_);

            // Create the rendering thread
            rendering_thread_ = std::thread(std::bind(&ObeliskMujocoRobot::SimRender, this));

            mujoco_setup_ = true;

            while (!activation_complete_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Starting Mujoco simulation loop.");

            // Set the initial condition based on a keyframe
            RCLCPP_INFO_STREAM(this->get_logger(), "Found " << this->model_->nkey << " Mujoco keyframes.");
            for (int i = 0; i < this->model_->nkey; i++) {
                std::string potential_keyframe(this->model_->names + this->model_->name_keyadr[i]);
                if (potential_keyframe == this->get_parameter("ic_keyframe").as_string()) {
                    RCLCPP_INFO_STREAM(this->get_logger(),
                                       "Setting initial condition to keyframe: " << potential_keyframe);
                    mj_resetDataKeyframe(model_, data_, i);
                    std::vector<double> shared_data_tmp;
                    for (int i = 0; i < model_->nu; i++) {
                        shared_data_tmp.push_back(data_->ctrl[i]);
                    }
                    SetSharedData(shared_data_tmp);
                }
            }

            while (!this->stop_thread_) {
                auto start_time = this->now();
                {
                    // Get the data from the shared vector safely
                    std::lock_guard<std::mutex> lock(shared_data_mut_);
                    for (int i = 0; i < nu_; i++) {
                        data_->ctrl[i] = shared_data_.at(i);
                    }
                }

                {
                    std::lock_guard<std::mutex> lock(sensor_data_mut_);
                    mj_step(model_, data_);
                }

                // Thread sleeping won't be precise enough for this, so instead we will spin lock the thread. This is
                // inefficient, but accurate.
                while ((this->now() - start_time).nanoseconds() < time_step_ * 1e9) {
                }
            }

            RCLCPP_WARN_STREAM(this->get_logger(), "Cleaning up simulation data and model...");
            // free MuJoCo model and data
            mj_deleteData(data_);
            mj_deleteModel(model_);

            rendering_thread_.join();
        }

      protected:
        void SimRender() {
            // Create the window
            if (!glfwInit()) {
                throw std::runtime_error("Could not initialize GLFW");
            }

            window_ =
                glfwCreateWindow(WINDOW_WIDTH_DEFAULT, WINDOW_LENGTH_DEFAULT, "Obelisk Mujoco Simulation", NULL, NULL);

            if (!window_) {
                throw std::runtime_error("Could not create the GLFW window!");
            }

            glfwMakeContextCurrent(window_);
            glfwSwapInterval(1);

            // initialize visualization data structures
            mjv_defaultFreeCamera(model_, &cam);
            mjv_defaultOption(&opt);
            mjv_defaultScene(&scn);
            mjr_defaultContext(&con);

            // Check if the user specified a custom camera
            this->template declare_parameter<std::string>("camera_name", "");
            const std::string camera_name = this->get_parameter("camera_name").as_string();
            int cam_id = mj_name2id(model_, mjOBJ_CAMERA, camera_name.c_str());
            if (cam_id != -1) {
                RCLCPP_INFO_STREAM(this->get_logger(), "camera name: " << camera_name);
                cam.type = mjCAMERA_FIXED;
                cam.fixedcamid = cam_id;
            }

            // create scene and context
            mjv_makeScene(model_, &scn, 2000);
            mjr_makeContext(model_, &con, mjFONTSCALE_150);

            // install GLFW mouse and keyboard callbacks
            glfwSetKeyCallback(window_, ObeliskMujocoRobot::KeyboardCallback);
            glfwSetCursorPosCallback(window_, ObeliskMujocoRobot::MouseMoveCallback);
            glfwSetMouseButtonCallback(window_, ObeliskMujocoRobot::MouseButtonCallback);
            glfwSetScrollCallback(window_, ObeliskMujocoRobot::ScrollCallback);

            while (!this->stop_thread_) {
                if (!glfwWindowShouldClose(window_)) {
                    // get framebuffer viewport
                    mjrRect viewport = {0, 0, 0, 0};
                    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

                    float time = 0;

                    // update scene and render
                    {
                        std::scoped_lock lock(sensor_data_mut_, shared_data_mut_);
                        time = data_->time;
                        mjv_updateScene(model_, data_, &opt, NULL, &cam, mjCAT_ALL, &scn);
                    }
                    mjr_render(viewport, &scn, &con);

                    // add time stamp in upper-left corner
                    char stamp[50];
                    sprintf_arr(stamp, "Time = %.3f", time);
                    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, stamp, NULL, &con);

                    // TODO (@zolkin): Add option to save the simulation render

                    // swap OpenGL buffers (blocking call due to v-sync)
                    glfwSwapBuffers(window_);

                    // process pending GUI events, call GLFW callbacks
                    glfwPollEvents();

                    std::this_thread::sleep_for(std::chrono::milliseconds(17)); // 58Hz render rate at most
                }
            }
            RCLCPP_WARN_STREAM(this->get_logger(), "Cleaning up simulation rendering...");
            // free visualization storage
            mjv_freeScene(&scn);
            mjr_freeContext(&con);
        }

        obelisk_sensor_msgs::msg::TrueSimState PublishTrueSimState() override {
            RCLCPP_WARN_STREAM_ONCE(this->get_logger(),
                                    "The true sim state currently only works for floating base robots!");
            static constexpr int DIM_FLOATING_BASE = 7;
            static constexpr int DIM_FLOATING_VEL  = 6;

            obelisk_sensor_msgs::msg::TrueSimState msg;

            std::lock_guard<std::mutex> lock(sensor_data_mut_);

            msg.header.frame_id = "world";
            msg.header.stamp    = this->now();

            // TODO: Base link name
            // TODO: Joint names
            // TODO: Handle both floating and fixed base

            // Configuration
            for (int i = 0; i < DIM_FLOATING_BASE; i++) {
                msg.q_base.emplace_back(this->data_->qpos[i]);
            }

            for (int i = DIM_FLOATING_BASE; i < this->model_->nq; i++) {
                msg.q_joints.emplace_back(this->data_->qpos[i]);
            }

            // Velocity
            for (int i = 0; i < DIM_FLOATING_VEL; i++) {
                msg.v_base.emplace_back(this->data_->qvel[i]);
            }

            for (int i = DIM_FLOATING_VEL; i < this->model_->nv; i++) {
                msg.v_joints.emplace_back(this->data_->qvel[i]);
            }

            this->template GetPublisher<obelisk_sensor_msgs::msg::TrueSimState>(this->state_pub_key_)->publish(msg);

            return msg;
        }

      private:
        // ---------------------------------------------- //
        // ----------- Multithreading Support ----------- //
        // ---------------------------------------------- //
        /**
         * @brief Safely retrieves the contents of shared data.
         *
         * @param data [output] where the shared data will be copied
         */
        void GetSharedData(std::vector<double>& data) {
            std::lock_guard<std::mutex> lock(shared_data_mut_);

            // Copy data out
            data = shared_data_;
        }

        /**
         * @brief Safely set the shared data.
         *
         * @param data the data to put into shared_data_
         */
        void SetSharedData(const std::vector<double>& data) {
            std::lock_guard<std::mutex> lock(shared_data_mut_);

            // Copy data in
            shared_data_ = data;
        }

        // Helper (see
        // https://github.com/google-deepmind/mujoco/blob/9107d3b507b2acf593d46c6cd268b5e3994bff6a/src/cc/array_safety.h#L64)
        template <std::size_t N> static inline int sprintf_arr(char (&dest)[N], const char* format, ...) {
            std::va_list vargs;
            va_start(vargs, format);
            int retval = std::vsnprintf(dest, N, format, vargs);
            va_end(vargs);
            return retval;
        }

        // --------------------------------------------- //
        // ----------- Config String Helpers ----------- //
        // --------------------------------------------- //
        std::filesystem::path GetXMLPath(const std::map<std::string, std::string>& config_map) {
            try {
                std::string path = config_map.at("model_xml_path");
                return std::filesystem::path(path);
            } catch (const std::exception& e) {
                throw std::runtime_error("No model XML path was provided!");
            }
        }

        std::string GetRobotPackage(const std::map<std::string, std::string>& config_map) {
            try {
                return config_map.at("robot_pkg");
            } catch (const std::exception& e) {
                return "None";
            }
        }

        int GetNumStepsPerViz(const std::map<std::string, std::string>& config_map) {
            try {
                int num_steps = std::stoi(config_map.at("num_steps_per_viz"));
                return num_steps;
            } catch (const std::exception& e) {
                return STEPS_PER_VIZ_DEFAULT;
            }
        }

        void ParseSensorString(std::string sensor_settings) {
            // Remove {} and []
            sensor_settings.erase(std::remove(sensor_settings.begin(), sensor_settings.end(), '{'),
                                  sensor_settings.end());
            sensor_settings.erase(std::remove(sensor_settings.begin(), sensor_settings.end(), '}'),
                                  sensor_settings.end());
            sensor_settings.erase(std::remove(sensor_settings.begin(), sensor_settings.end(), '['),
                                  sensor_settings.end());
            sensor_settings.erase(std::remove(sensor_settings.begin(), sensor_settings.end(), ']'),
                                  sensor_settings.end());

            // Break the string by the "+"s.
            const std::string group_delim = "+";
            std::vector<std::string> sensor_groups;
            while (!sensor_settings.empty()) {
                size_t plus_idx = sensor_settings.find(group_delim);
                if (plus_idx == std::string::npos) {
                    plus_idx = sensor_settings.length();
                }
                sensor_groups.emplace_back(sensor_settings.substr(0, plus_idx));
                sensor_settings.erase(0, plus_idx + group_delim.length());
            }

            // Wait for mujoco setup to complete
            while (!mujoco_setup_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            num_sensors_ = 0;

            callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            for (auto group : sensor_groups) {
                // Create a new map between the names and their string values
                const std::string setting_delim = "|";
                const std::string val_delim     = "=";
                std::map<std::string, std::string> setting_map;

                size_t val_idx;
                while (!group.empty()) {
                    val_idx = group.find(setting_delim);
                    if (val_idx == std::string::npos) {
                        val_idx = group.length();
                    }

                    size_t setting_idx = group.find(val_delim);
                    if (setting_idx == std::string::npos) {
                        throw std::runtime_error("Invalid sensor setting string!");
                    }

                    std::string setting = group.substr(0, setting_idx);
                    std::string val     = group.substr(setting_idx + 1, val_idx - (setting_idx + 1));
                    setting_map.emplace(setting, val);
                    group.erase(0, val_idx + val_delim.length());
                }

                // The dt for the timer
                double dt = -1;
                try {
                    dt = std::stod(setting_map.at("dt"));
                    if (dt <= 0) {
                        throw std::runtime_error("Invalid dt. Must be > 0.");
                    }
                } catch (const std::exception& e) {
                    throw std::runtime_error("No dt provided for a sensor!");
                }

                // The topic for the publisher
                std::string topic;
                try {
                    topic = setting_map.at("topic");
                } catch (const std::exception& e) {
                    throw std::runtime_error("No topic provided for a sensor!");
                }

                const int depth = this->ObeliskNode::GetHistoryDepth(setting_map);

                std::string sensor_type;
                try {
                    sensor_type = setting_map.at("msg_type");
                } catch (const std::exception& e) {
                    throw std::runtime_error("No sensor type provided for a sensor!");
                }

                std::vector<std::string> sensor_names;
                std::vector<std::string> mj_sensor_types;
                const std::string name_delim = "&";
                const std::string type_delim = "$";
                try {
                    std::string names = setting_map.at("sensor_names");
                    while (!names.empty()) {
                        size_t plus_idx = names.find(name_delim);
                        if (plus_idx == std::string::npos) {
                            plus_idx = names.length();
                        }
                        // now determine sensor type
                        size_t dollar_idx = names.find(type_delim);
                        if (dollar_idx == std::string::npos) {
                            dollar_idx = names.length();
                        }
                        sensor_names.emplace_back(names.substr(0, dollar_idx));
                        mj_sensor_types.emplace_back(names.substr(dollar_idx + type_delim.length(),
                                                                  plus_idx - dollar_idx - type_delim.length()));
                        names.erase(0, plus_idx + name_delim.length());
                    }
                } catch (const std::exception& e) {
                    throw std::runtime_error("No sensor name was provided for a sensor group!");
                }

                // Create the timer and publishers with the settings
                // Make a key
                const std::string sensor_key = "sensor_group_" + std::to_string(num_sensors_);
                if (sensor_type == obelisk_sensor_msgs::msg::ObkJointEncoders::MESSAGE_NAME) {
                    // Make a publisher and add it to the list
                    auto pub = ObeliskNode::create_publisher<obelisk_sensor_msgs::msg::ObkJointEncoders>(topic, depth);
                    this->publishers_[sensor_key] =
                        std::make_shared<internal::ObeliskPublisher<obelisk_sensor_msgs::msg::ObkJointEncoders>>(pub);

                    // Add the timer to the list
                    this->timers_[sensor_key] = this->create_wall_timer(
                        std::chrono::milliseconds(static_cast<uint>(1e3 * dt)),
                        CreateTimerCallback<obelisk_sensor_msgs::msg::ObkJointEncoders>(
                            sensor_names, mj_sensor_types,
                            this->template GetPublisher<obelisk_sensor_msgs::msg::ObkJointEncoders>(sensor_key)),
                        callback_group_);
                } else if (sensor_type == obelisk_sensor_msgs::msg::ObkImu::MESSAGE_NAME) {
                    // Make a publisher and add it to the list
                    auto pub = ObeliskNode::create_publisher<obelisk_sensor_msgs::msg::ObkImu>(topic, depth);
                    this->publishers_[sensor_key] =
                        std::make_shared<internal::ObeliskPublisher<obelisk_sensor_msgs::msg::ObkImu>>(pub);

                    // Add the timer to the list
                    this->timers_[sensor_key] = this->create_wall_timer(
                        std::chrono::milliseconds(static_cast<uint>(1e3 * dt)),
                        CreateTimerCallback<obelisk_sensor_msgs::msg::ObkImu>(
                            sensor_names, mj_sensor_types,
                            this->template GetPublisher<obelisk_sensor_msgs::msg::ObkImu>(sensor_key)),
                        callback_group_);
                } else if (sensor_type == "PoseStamped") {
                    // Make a publisher and add it to the list
                    auto pub = ObeliskNode::create_publisher<geometry_msgs::msg::PoseStamped>(topic, depth);
                    this->publishers_[sensor_key] =
                        std::make_shared<internal::ObeliskPublisher<geometry_msgs::msg::PoseStamped>>(pub);

                    // Add the timer to the list
                    this->timers_[sensor_key] = this->create_wall_timer(
                        std::chrono::milliseconds(static_cast<uint>(1e3 * dt)),
                        CreateTimerCallback<geometry_msgs::msg::PoseStamped>(
                            sensor_names, mj_sensor_types,
                            this->template GetPublisher<geometry_msgs::msg::PoseStamped>(sensor_key)),
                        callback_group_);
                } else if (sensor_type == "Odometry") {
                    // Make a publisher and add it to the list
                    auto pub = ObeliskNode::create_publisher<nav_msgs::msg::Odometry>(topic, depth);
                    this->publishers_[sensor_key] =
                        std::make_shared<internal::ObeliskPublisher<nav_msgs::msg::Odometry>>(pub);

                    // Add the timer to the list
                    this->timers_[sensor_key] = this->create_wall_timer(
                        std::chrono::milliseconds(static_cast<uint>(1e3 * dt)),
                        CreateTimerCallback<nav_msgs::msg::Odometry>(
                            sensor_names, mj_sensor_types,
                            this->template GetPublisher<nav_msgs::msg::Odometry>(sensor_key)),
                        callback_group_);
                } else if (sensor_type == "GridCells") {
                    // Make a publisher and add it to the list
                    auto pub = ObeliskNode::create_publisher<nav_msgs::msg::GridCells>(topic, depth);
                    this->publishers_[sensor_key] =
                        std::make_shared<internal::ObeliskPublisher<nav_msgs::msg::GridCells>>(pub);

                    // Add the timer to the list
                    this->timers_[sensor_key] = this->create_wall_timer(
                        std::chrono::milliseconds(static_cast<uint>(1e3 * dt)),
                        CreateTimerCallback<nav_msgs::msg::GridCells>(
                            sensor_names, mj_sensor_types,
                            this->template GetPublisher<nav_msgs::msg::GridCells>(sensor_key)),
                        callback_group_);
                } else {
                    throw std::runtime_error("Sensor type not supported!");
                }

                this->timers_[sensor_key]->cancel(); // Stop the timer
                num_sensors_++;
            }
        }

        /**
         * @brief Parse the settings for visualizing geometries from the mujoco scene.
         */
        void ParseVizString(std::string viz_settings) {
            // TODO: We should probably make the yaml entry a normal list, but with the way config strings are done that
            // will be a huge pain So in the future we should adjust this. This is because I don't actually need the
            // user to provide the geom type.

            // Remove {} and []
            viz_settings.erase(std::remove(viz_settings.begin(), viz_settings.end(), '{'), viz_settings.end());
            viz_settings.erase(std::remove(viz_settings.begin(), viz_settings.end(), '}'), viz_settings.end());
            viz_settings.erase(std::remove(viz_settings.begin(), viz_settings.end(), '['), viz_settings.end());
            viz_settings.erase(std::remove(viz_settings.begin(), viz_settings.end(), ']'), viz_settings.end());

            // Wait for mujoco setup to complete
            while (!mujoco_setup_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            // Extract the geom id's for publishing
            // Create a new map between the names and their string values
            const std::string setting_delim = "|";
            const std::string val_delim     = ":";
            std::map<std::string, std::string> setting_map;

            size_t val_idx;
            while (!viz_settings.empty()) {
                val_idx = viz_settings.find(setting_delim);
                if (val_idx == std::string::npos) {
                    val_idx = viz_settings.length();
                }

                size_t setting_idx = viz_settings.find(val_delim);
                if (setting_idx == std::string::npos) {
                    throw std::runtime_error("Invalid viz setting string!");
                }

                std::string setting = viz_settings.substr(0, setting_idx);
                std::string val     = viz_settings.substr(setting_idx + 1, val_idx - (setting_idx + 1));
                setting_map.emplace(setting, val);
                viz_settings.erase(0, val_idx + val_delim.length());
            }

            // The dt for the timer
            double dt = -1;
            try {
                dt = std::stod(setting_map.at("dt"));
                if (dt <= 0) {
                    throw std::runtime_error("Invalid dt. Must be > 0.");
                }
            } catch (const std::exception& e) {
                throw std::runtime_error("No dt provided for the viz publisher!");
            }

            for (const auto& [key, value] : setting_map) {
                if (key != "dt") {
                    // Find the mujoco geom with that name
                    const int geom_id = mj_name2id(model_, mjtObj::mjOBJ_GEOM, key.c_str());
                    if (geom_id == -1) {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "Geom " << key << " not found in the model! Skipping!");
                    } else {
                        // TODO: Can easily add elipsoids in
                        if (model_->geom_type[geom_id] != mjtGeom::mjGEOM_BOX &&
                            model_->geom_type[geom_id] != mjtGeom::mjGEOM_SPHERE &&
                            model_->geom_type[geom_id] != mjtGeom::mjGEOM_CYLINDER) {
                            RCLCPP_ERROR_STREAM(this->get_logger(),
                                                "Geom type not supported! Only support Box, Sphere, and Cylinder.");
                        } else {
                            viz_geoms_.emplace_back(geom_id);
                        }
                    }
                }
            }

            if (viz_geoms_.size() > 0) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Publishing the given mujoco geoms.");
                // For now hard code the topic
                auto pub = ObeliskNode::create_publisher<visualization_msgs::msg::MarkerArray>(
                    "obelisk/sim/mujoco_scene_viz", 10);
                this->publishers_[viz_key_] =
                    std::make_shared<internal::ObeliskPublisher<visualization_msgs::msg::MarkerArray>>(pub);

                // Add the timer to the list
                this->timers_[viz_key_] =
                    this->create_wall_timer(std::chrono::milliseconds(static_cast<uint>(1e3 * dt)),
                                            std::bind(&ObeliskMujocoRobot::VizTimerCallback, this), callback_group_);
            }
        }

        void VizTimerCallback() {
            std::vector<std::array<double, 3>> positions(viz_geoms_.size());
            std::vector<std::array<double, 9>> rotmats(viz_geoms_.size());
            std::vector<std::array<double, 3>> sizes(viz_geoms_.size());
            std::vector<int> geom_types(viz_geoms_.size());

            {
                // Grab the mutex on the model and data
                std::lock_guard<std::mutex> lock(sensor_data_mut_);

                // Get the position and orientation of the geoms
                for (size_t i = 0; i < viz_geoms_.size(); i++) {
                    geom_types[i] = model_->geom_type[viz_geoms_[i]];

                    for (int j = 0; j < 3; j++) {
                        positions[i][j] = data_->geom_xpos[3 * viz_geoms_[i] + j];
                        sizes[i][j]     = model_->geom_size[3 * viz_geoms_[i] + j];
                    }

                    for (int j = 0; j < 9; j++) {
                        rotmats[i][j] = data_->geom_xmat[9 * viz_geoms_[i] + j];
                    }
                }
            }

            // Format for the display marker
            visualization_msgs::msg::MarkerArray msg;
            msg.markers.resize(viz_geoms_.size());

            for (size_t i = 0; i < msg.markers.size(); i++) {
                msg.markers[i].header.frame_id = "world";
                msg.markers[i].header.stamp    = this->now();

                msg.markers[i].ns     = "mujoco_scene_viz";
                msg.markers[i].id     = i;
                msg.markers[i].action = visualization_msgs::msg::Marker::MODIFY;

                msg.markers[i].pose.position.x = positions[i][0];
                msg.markers[i].pose.position.y = positions[i][1];
                msg.markers[i].pose.position.z = positions[i][2];

                // Convert the rotmats to quats
                mjtNum quat[4];
                mjtNum* mat = rotmats[i].data();
                mju_mat2Quat(quat, mat);

                msg.markers[i].pose.orientation.x = -quat[1];
                msg.markers[i].pose.orientation.y = -quat[2];
                msg.markers[i].pose.orientation.z = -quat[3];
                msg.markers[i].pose.orientation.w = -quat[0];

                msg.markers[i].color.a = 1.0;
                msg.markers[i].color.r = 0.69;
                msg.markers[i].color.g = 0.541;
                msg.markers[i].color.b = 0.094;

                if (geom_types[i] == mjtGeom::mjGEOM_BOX) {
                    msg.markers[i].scale.x = sizes[i][0] * 2;
                    msg.markers[i].scale.y = sizes[i][1] * 2;
                    msg.markers[i].scale.z = sizes[i][2] * 2;

                    msg.markers[i].type = visualization_msgs::msg::Marker::CUBE;
                } else if (geom_types[i] == mjtGeom::mjGEOM_SPHERE) {
                    msg.markers[i].scale.x = sizes[i][0] * 2;
                    msg.markers[i].scale.y = sizes[i][0] * 2;
                    msg.markers[i].scale.z = sizes[i][0] * 2;

                    msg.markers[i].type = visualization_msgs::msg::Marker::SPHERE;
                } else if (geom_types[i] == mjtGeom::mjGEOM_CYLINDER) {
                    msg.markers[i].scale.x = sizes[i][0] * 2;
                    msg.markers[i].scale.y = sizes[i][0] * 2;
                    msg.markers[i].scale.z = sizes[i][1] * 2;

                    msg.markers[i].type = visualization_msgs::msg::Marker::CYLINDER;
                }
            }

            // Publish
            this->template GetPublisher<visualization_msgs::msg::MarkerArray>(viz_key_)->publish(msg);
        }

        /**
         * @brief Create all the sensor message callbacks.
         * @param MessageT the templated message type
         * @param sensor_names a list of mujoco sensors associated with this message
         * @param sensor_types a list of mujoco sensor types associated the sensor_names
         * @param publisher the publisher to use
         *
         * @return the function pointer to the callback
         */
        template <typename MessageT>
        std::function<void()>
        CreateTimerCallback(const std::vector<std::string>& sensor_names,
                            const std::vector<std::string>& mj_sensor_types,
                            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT>> publisher) {
            if constexpr (std::is_same<MessageT, obelisk_sensor_msgs::msg::ObkJointEncoders>::value) {
                // Check that joints with encoders are only hinge or slide.
                for (size_t i = 0; i < sensor_names.size(); i++) {
                    std::lock_guard<std::mutex> lock(sensor_data_mut_);
                    int sensor_id = mj_name2id(this->model_, mjOBJ_SENSOR, sensor_names.at(i).c_str());
                    if (sensor_id == -1) {
                        RCLCPP_ERROR_STREAM_ONCE(
                            this->get_logger(),
                            "Sensor not found in Mujoco! Make sure your XML has the sensor. Sensor name: "
                                << sensor_names.at(i));
                        throw std::runtime_error("Sensor not found in Mujoco! Make sure your XML has the sensor.");
                    }
                    int joint_id   = this->model_->sensor_objid[sensor_id];
                    int joint_type = this->model_->jnt_type[joint_id];
                    if (joint_type != mjJNT_HINGE && joint_type != mjJNT_SLIDE) {
                        throw std::runtime_error("Joint encoder sensor attached to a non-hinge or non-slide joint!");
                    }
                }

                // --------------------------------------------- //
                // ---------- Joint encoder call back ---------- //
                // --------------------------------------------- //
                auto cb = [publisher, sensor_names, mj_sensor_types, this]() {
                    obelisk_sensor_msgs::msg::ObkJointEncoders msg;

                    std::lock_guard<std::mutex> lock(sensor_data_mut_);
                    for (size_t i = 0; i < sensor_names.size(); i++) {
                        int sensor_id = mj_name2id(this->model_, mjOBJ_SENSOR, sensor_names.at(i).c_str());
                        if (sensor_id == -1) {
                            throw std::runtime_error("Sensor not found in Mujoco! Make sure your XML has the sensor.");
                        }

                        // *** Note *** The joint names are in the order provided by the joint_pos sensors.
                        //  If the velocity sensor ordering does not match the position sensors, then their joint names
                        //  will not align.
                        int sensor_addr = this->model_->sensor_adr[sensor_id];
                        if (mj_sensor_types.at(i) == "jointpos") {
                            msg.joint_pos.emplace_back(this->data_->sensordata[sensor_addr]);
                            int joint_id = this->model_->sensor_objid[sensor_id];
                            if (joint_id == -1) {
                                RCLCPP_ERROR_STREAM(this->get_logger(), "Joint associated with "
                                                                            << sensor_names.at(i)
                                                                            << " is not found in the Mujoco xml.");
                            } else {
                                msg.joint_names.emplace_back(this->model_->names + this->model_->name_jntadr[joint_id]);
                            }
                        } else if (mj_sensor_types.at(i) == "jointvel") {
                            msg.joint_vel.emplace_back(this->data_->sensordata[sensor_addr]);
                        } else {
                            RCLCPP_ERROR_STREAM(
                                this->get_logger(),
                                "Sensor " << sensor_names.at(i)
                                          << " is not associated with a valid Mujoco sensor type! Current sensor type: "
                                          << mj_sensor_types.at(i));
                        }

                        msg.header.stamp = this->now();
                    }
                    publisher->publish(msg);
                };

                RCLCPP_INFO_STREAM(this->get_logger(), "Timer callback created for JointEncoders!");
                return cb;
            } else if constexpr (std::is_same<MessageT, obelisk_sensor_msgs::msg::ObkImu>::value) {
                // ----------------------------------- //
                // ---------- IMU call back ---------- //
                // ----------------------------------- //
                auto cb = [publisher, sensor_names, mj_sensor_types, this]() {
                    obelisk_sensor_msgs::msg::ObkImu msg;

                    // TODO: For now all the covariances are 0
                    // *** Note *** Mujoco does not support adding noise natively since v3.14.
                    // TODO: Consider adding noise here ourselves

                    std::lock_guard<std::mutex> lock(sensor_data_mut_);
                    bool has_acc       = false;
                    bool has_gyro      = false;
                    bool has_framequat = false;

                    for (size_t i = 0; i < sensor_names.size(); i++) {
                        int sensor_id = mj_name2id(this->model_, mjOBJ_SENSOR, sensor_names.at(i).c_str());
                        if (sensor_id == -1) {
                            throw std::runtime_error("Sensor not found in Mujoco! Make sure your XML has the sensor.");
                        }

                        // *** Note *** We only use the frame associated with the accelerometer. If the gyro or
                        // magnetometer is at a different frame, then it needs to be its own sensor. Be sure the gyro,
                        // magnetometer, and accelerometer are at the same frame.
                        int sensor_addr = this->model_->sensor_adr[sensor_id];
                        if (mj_sensor_types.at(i) == "accelerometer") {
                            if (!has_acc) {
                                msg.linear_acceleration.x = this->data_->sensordata[sensor_addr];
                                msg.linear_acceleration.y = this->data_->sensordata[sensor_addr + 1];
                                msg.linear_acceleration.z = this->data_->sensordata[sensor_addr + 2];

                                // Accelerometers are always mounted to sites
                                int site_id = this->model_->sensor_objid[sensor_id];
                                msg.header.frame_id =
                                    std::string(this->model_->names + this->model_->name_siteadr[site_id]);

                                has_acc = true;
                            } else {
                                RCLCPP_ERROR_STREAM(this->get_logger(),
                                                    "There are two accelerometers associated with this IMU! Ignoring "
                                                        << sensor_names.at(i));
                            }
                        } else if (mj_sensor_types.at(i) == "gyro") {
                            if (!has_gyro) {
                                msg.angular_velocity.x = this->data_->sensordata[sensor_addr];
                                msg.angular_velocity.y = this->data_->sensordata[sensor_addr + 1];
                                msg.angular_velocity.z = this->data_->sensordata[sensor_addr + 2];
                                has_gyro               = true;
                            } else {
                                RCLCPP_ERROR_STREAM(this->get_logger(),
                                                    "There are two gyros associated with this IMU! Ignoring "
                                                        << sensor_names.at(i));
                            }
                        } else if (mj_sensor_types.at(i) == "framequat") {
                            if (!has_framequat) {
                                msg.orientation.w = this->data_->sensordata[sensor_addr];
                                msg.orientation.x = this->data_->sensordata[sensor_addr + 1];
                                msg.orientation.y = this->data_->sensordata[sensor_addr + 2];
                                msg.orientation.z = this->data_->sensordata[sensor_addr + 3];
                                has_framequat     = true;
                            } else {
                                RCLCPP_ERROR_STREAM(this->get_logger(),
                                                    "There are two framequats associated with this IMU! Ignoring "
                                                        << sensor_names.at(i));
                            }
                        } else {
                            RCLCPP_ERROR_STREAM(
                                this->get_logger(),
                                "Sensor " << sensor_names.at(i)
                                          << " is not associated with a valid Mujoco sensor type! Current sensor type: "
                                          << mj_sensor_types.at(i));
                        }

                        msg.header.stamp = this->now();
                    }
                    publisher->publish(msg);
                };

                RCLCPP_INFO_STREAM(this->get_logger(), "Timer callback created for a IMU!");
                return cb;
            } else if constexpr (std::is_same<MessageT, geometry_msgs::msg::PoseStamped>::value) {
                // ------------------------------------------ //
                // ---------- Frame Pose call back ---------- //
                // ------------------------------------------ //
                auto cb = [publisher, sensor_names, mj_sensor_types, this]() {
                    geometry_msgs::msg::PoseStamped msg;

                    bool has_framepos  = false;
                    bool has_framequat = false;

                    std::lock_guard<std::mutex> lock(sensor_data_mut_);
                    for (size_t i = 0; i < sensor_names.size(); i++) {
                        int sensor_id = mj_name2id(this->model_, mjOBJ_SENSOR, sensor_names.at(i).c_str());
                        if (sensor_id == -1) {
                            throw std::runtime_error("Sensor not found in Mujoco! Make sure your XML has the sensor.");
                        }

                        // *** Note *** We only use the frame name and relative frame for the framepos. Be sure that
                        // framepos and framequat have the same object/frame. Framequat is always given in global
                        // coordinates
                        int sensor_addr = this->model_->sensor_adr[sensor_id];
                        if (mj_sensor_types.at(i) == "framepos") {
                            if (!has_framepos) {
                                msg.pose.position.x = this->data_->sensordata[sensor_addr];
                                msg.pose.position.y = this->data_->sensordata[sensor_addr + 1];
                                msg.pose.position.z = this->data_->sensordata[sensor_addr + 2];

                                // // Framepos sensors can be mounted to different objects
                                // int obj_type = this->model_->sensor_objtype[sensor_id];
                                // int obj_id   = this->model_->sensor_objid[sensor_id];
                                // if (obj_type == mjOBJ_SITE) {
                                //     msg.frame_name =
                                //         std::string(this->model_->names + this->model_->name_siteadr[obj_id]);
                                // } else if (obj_type == mjOBJ_BODY) {
                                //     msg.frame_name =
                                //         std::string(this->model_->names + this->model_->name_bodyadr[obj_id]);
                                // } else if (obj_type == mjOBJ_GEOM) {
                                //     msg.frame_name =
                                //         std::string(this->model_->names + this->model_->name_geomadr[obj_id]);
                                // } else if (obj_type == mjOBJ_CAMERA) {
                                //     msg.frame_name =
                                //         std::string(this->model_->names + this->model_->name_camadr[obj_id]);
                                // } else {
                                //     RCLCPP_ERROR_STREAM(
                                //         this->get_logger(),
                                //         "Framepos sensor, "
                                //             << sensor_names.at(i)
                                //             << ", is not associated with a supported Mujoco object type! "
                                //                "Current object type (mjtObj): "
                                //             << obj_type);
                                // }

                                if (this->model_->sensor_refid[sensor_id] == -1) {
                                    msg.header.frame_id = "world"; // TODO: Consider not hard-coding this
                                } else {
                                    int ref_type = this->model_->sensor_reftype[sensor_id];
                                    int ref_id   = this->model_->sensor_refid[sensor_id];
                                    if (ref_type == mjOBJ_SITE) {
                                        msg.header.frame_id =
                                            std::string(this->model_->names + this->model_->name_siteadr[ref_id]);
                                    } else if (ref_type == mjOBJ_BODY) {
                                        msg.header.frame_id =
                                            std::string(this->model_->names + this->model_->name_bodyadr[ref_id]);
                                    } else if (ref_type == mjOBJ_GEOM) {
                                        msg.header.frame_id =
                                            std::string(this->model_->names + this->model_->name_geomadr[ref_id]);
                                    } else if (ref_type == mjOBJ_CAMERA) {
                                        msg.header.frame_id =
                                            std::string(this->model_->names + this->model_->name_camadr[ref_id]);
                                    } else {
                                        RCLCPP_ERROR_STREAM(this->get_logger(),
                                                            "Framepos sensor, "
                                                                << sensor_names.at(i)
                                                                << ", is not associated with a supported Mujoco object "
                                                                   "type! Current object type (mjtObj): "
                                                                << ref_type);
                                    }
                                }
                                has_framepos = true;
                            } else {
                                RCLCPP_ERROR_STREAM(this->get_logger(),
                                                    "There are two framepos associated with this FramePose! Ignoring "
                                                        << sensor_names.at(i));
                            }
                        } else if (mj_sensor_types.at(i) == "framequat") {
                            if (!has_framequat) {
                                msg.pose.orientation.w = this->data_->sensordata[sensor_addr];
                                msg.pose.orientation.x = this->data_->sensordata[sensor_addr + 1];
                                msg.pose.orientation.y = this->data_->sensordata[sensor_addr + 2];
                                msg.pose.orientation.z = this->data_->sensordata[sensor_addr + 3];
                                has_framequat     = true;
                            } else {
                                RCLCPP_ERROR_STREAM(this->get_logger(),
                                                    "There are two framequats associated with this FramePose! Ignoring "
                                                        << sensor_names.at(i));
                            }
                        } else {
                            RCLCPP_ERROR_STREAM(
                                this->get_logger(),
                                "Sensor " << sensor_names.at(i)
                                          << " is not associated with a valid Mujoco sensor type! Current sensor type: "
                                          << mj_sensor_types.at(i));
                        }

                        msg.header.stamp = this->now();
                    }
                    publisher->publish(msg);
                };

                RCLCPP_INFO_STREAM(this->get_logger(), "Timer callback created for a FramePose!");
                return cb;
            } else if constexpr (std::is_same<MessageT, nav_msgs::msg::Odometry>::value) {
                // ------------------------------------------ //
                // ------------ Odometry Sensor ------------- //
                // ------------------------------------------ //
                auto cb = [publisher, sensor_names, mj_sensor_types, this]() {
                    // This sensor is always made up of:
                    //  - Framepos
                    //  - Framequat
                    //  - velocimeter
                    //  - gyro
                    // NOTE: For now the velocities are in the local frame, we should add an option to make this sensor with the global ones

                    nav_msgs::msg::Odometry msg;

                    bool has_framepos  = false;
                    bool has_framequat = false;
                    bool has_velocimeter = false;
                    bool has_gyro = false;

                    std::lock_guard<std::mutex> lock(sensor_data_mut_);
                    for (size_t i = 0; i < sensor_names.size(); i++) {
                        int sensor_id = mj_name2id(this->model_, mjOBJ_SENSOR, sensor_names.at(i).c_str());
                        if (sensor_id == -1) {
                            throw std::runtime_error("Sensor not found in Mujoco! Make sure your XML has the sensor: " + sensor_names.at(i));
                        }

                        // *** Note *** We only use the frame name and relative frame for the framepos. Be sure that
                        // framepos and framequat have the same object/frame. Framequat is always given in global
                        // coordinates
                        int sensor_addr = this->model_->sensor_adr[sensor_id];
                        if (mj_sensor_types.at(i) == "framepos") {
                            if (!has_framepos) {
                                msg.pose.pose.position.x = this->data_->sensordata[sensor_addr];
                                msg.pose.pose.position.y = this->data_->sensordata[sensor_addr + 1];
                                msg.pose.pose.position.z = this->data_->sensordata[sensor_addr + 2];

                                if (this->model_->sensor_refid[sensor_id] == -1) {
                                    msg.header.frame_id = "world"; // TODO: Consider not hard-coding this
                                } else {
                                    int ref_type = this->model_->sensor_reftype[sensor_id];
                                    int ref_id   = this->model_->sensor_refid[sensor_id];
                                    if (ref_type == mjOBJ_SITE) {
                                        msg.header.frame_id =
                                            std::string(this->model_->names + this->model_->name_siteadr[ref_id]);
                                    } else if (ref_type == mjOBJ_BODY) {
                                        msg.header.frame_id =
                                            std::string(this->model_->names + this->model_->name_bodyadr[ref_id]);
                                    } else if (ref_type == mjOBJ_GEOM) {
                                        msg.header.frame_id =
                                            std::string(this->model_->names + this->model_->name_geomadr[ref_id]);
                                    } else if (ref_type == mjOBJ_CAMERA) {
                                        msg.header.frame_id =
                                            std::string(this->model_->names + this->model_->name_camadr[ref_id]);
                                    } else {
                                        RCLCPP_ERROR_STREAM(this->get_logger(),
                                                            "Framepos sensor, "
                                                                << sensor_names.at(i)
                                                                << ", is not associated with a supported Mujoco object "
                                                                   "type! Current object type (mjtObj): "
                                                                << ref_type);
                                    }
                                }
                                has_framepos = true;
                            } else {
                                RCLCPP_ERROR_STREAM(this->get_logger(),
                                                    "There are two framepos associated with this FramePose! Ignoring "
                                                        << sensor_names.at(i));
                            }
                        } else if (mj_sensor_types.at(i) == "framequat") {
                            if (!has_framequat) {
                                msg.pose.pose.orientation.w = this->data_->sensordata[sensor_addr];
                                msg.pose.pose.orientation.x = this->data_->sensordata[sensor_addr + 1];
                                msg.pose.pose.orientation.y = this->data_->sensordata[sensor_addr + 2];
                                msg.pose.pose.orientation.z = this->data_->sensordata[sensor_addr + 3];
                                has_framequat     = true;
                            } else {
                                RCLCPP_ERROR_STREAM(this->get_logger(),
                                                    "There are two framequats associated with this FramePose! Ignoring "
                                                        << sensor_names.at(i));
                            }
                        } else if (mj_sensor_types.at(i) == "velocimeter") {
                            if (!has_velocimeter) {
                                msg.twist.twist.linear.x = this->data_->sensordata[sensor_addr];
                                msg.twist.twist.linear.y = this->data_->sensordata[sensor_addr + 1];
                                msg.twist.twist.linear.z = this->data_->sensordata[sensor_addr + 2];

                                has_velocimeter = true;
                            }
                        } else if (mj_sensor_types.at(i) == "gyro") {
                            if (!has_gyro) {
                                msg.twist.twist.angular.x = this->data_->sensordata[sensor_addr];
                                msg.twist.twist.angular.y = this->data_->sensordata[sensor_addr + 1];
                                msg.twist.twist.angular.z = this->data_->sensordata[sensor_addr + 2];

                                has_gyro = true;
                            }
                        } else {
                            RCLCPP_ERROR_STREAM(
                                this->get_logger(),
                                "Sensor " << sensor_names.at(i)
                                          << " is not associated with a valid Mujoco sensor type! Current sensor type: "
                                          << mj_sensor_types.at(i));
                        }

                        msg.header.stamp = this->now();
                    }
                    publisher->publish(msg);
                };

                RCLCPP_INFO_STREAM(this->get_logger(), "Timer callback created for a Odometry!");
                return cb;
            } else if constexpr (std::is_same<MessageT, nav_msgs::msg::GridCells>::value) {
                // ------------------------------------------ //
                // ------------ Scan Dots Sensor ------------ //
                // ------------------------------------------ //
                auto cb = [publisher, sensor_names, mj_sensor_types, this]() {
                    // This sensor is always made up of:
                    //  - Framepos

                    nav_msgs::msg::GridCells msg;

                    bool has_framepos  = false;
                    
                    // Verify that the proper parameters have been passed
                    if (this->height_map_grid_size_.size() != 2) {
                        throw std::runtime_error("Attempted to use a scan dots sensor without providing a valid grid size!");
                    }
                    int x_rays = this->height_map_grid_size_[0] / this->height_map_grid_spacing_ + 1;
                    int y_rays = this->height_map_grid_size_[1] / this->height_map_grid_spacing_ + 1;
                    msg.cells.resize(x_rays * y_rays);

                    if (this->height_map_grid_spacing_ == 0) {
                        throw std::runtime_error("Attempted to use a scan dots sensor without providing a valid grid spacing!");
                    }


                    std::array<int, 2> x_y_num_rays = {x_rays, y_rays};

                    msg.cell_width = this->height_map_grid_spacing_;

                    std::lock_guard<std::mutex> lock(sensor_data_mut_);
                    for (size_t i = 0; i < sensor_names.size(); i++) {
                        int sensor_id = mj_name2id(this->model_, mjOBJ_SENSOR, sensor_names.at(i).c_str());
                        if (sensor_id == -1) {
                            throw std::runtime_error("Sensor not found in Mujoco! Make sure your XML has the sensor: " + sensor_names.at(i));
                        }
                        // Get the sensor id
                        int sensor_addr = this->model_->sensor_adr[sensor_id];

                        if (mj_sensor_types.at(i) == "framepos") {
                            if (!has_framepos) {
                                // Starting ray origin position (top-left corner of scan)
                                std::array<double, 3> sensor_pos = {this->data_->sensordata[sensor_addr],
                                    this->data_->sensordata[sensor_addr + 1],
                                    this->data_->sensordata[sensor_addr + 2] + 10.0 };  // shift upward

                                int site_id = model_->sensor_objid[sensor_id];
                                std::array<double, 3> site_pos_global = {this->data_->site_xpos[3*site_id],
                                    this->data_->site_xpos[3*site_id + 1],
                                    this->data_->site_xpos[3*site_id + 2],
                                };

                                sensor_pos[0] += site_pos_global[0];
                                sensor_pos[1] += site_pos_global[1];

                                // Get the yaw from the quat
                                double w = this->data_->qpos[3], x = this->data_->qpos[4], y = this->data_->qpos[5], z = this->data_->qpos[6];

                                // Yaw (Z-axis rotation)
                                double siny_cosp = 2.0 * (w * z + x * y);
                                double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
                                double base_yaw = std::atan2(siny_cosp, cosy_cosp);

                                // Make the rotation matrix
                                double cos_yaw = std::cos(base_yaw);
                                double sin_yaw = std::sin(base_yaw);
                                double R_2d[2][2] = {
                                    { cos_yaw, -sin_yaw },
                                    { sin_yaw,  cos_yaw }
                                };

                                // Adjust X and Y to go to bottom-left corner
                                std::array<double, 2> offset = { -this->height_map_grid_size_[0] / 2.0, -this->height_map_grid_size_[1] / 2.0 };
                                // Rotate offset: R_2d x offset
                                std::array<double, 2> rotated_offset = {
                                    R_2d[0][0] * offset[0] + R_2d[0][1] * offset[1],
                                    R_2d[1][0] * offset[0] + R_2d[1][1] * offset[1]
                                };
                                sensor_pos[0] += rotated_offset[0];
                                sensor_pos[1] += rotated_offset[1];

                                // Ray direction: straight down
                                const std::array<double, 3> direction = { 0.0, 0.0, -1.0 };

                                // Geom groups active for ray collisions
                                mjtByte geom_group[mjNGROUP] = {0, 0, 0, 0, 0, 0};
                                for (size_t i = 0; i < this->height_map_geom_group_.size(); i++) {
                                    geom_group[this->height_map_geom_group_[i]] = 1;
                                }

                                // Temp storage for geom id output
                                int geom_id[1] = { -1 };

                                int ii = 0;
                                for (int x = 0; x < x_y_num_rays[0]; ++x) {
                                    for (int y = 0; y < x_y_num_rays[1]; ++y) {
                                        // Compute origin for this ray
                                        offset = { this->height_map_grid_spacing_ * x, this->height_map_grid_spacing_ * y};

                                        // Rotate offset: R_2d x offset
                                        rotated_offset = {
                                            R_2d[0][0] * offset[0] + R_2d[0][1] * offset[1],
                                            R_2d[1][0] * offset[0] + R_2d[1][1] * offset[1]
                                        };

                                        std::array<double, 3> ray_origin = {
                                            sensor_pos[0] + rotated_offset[0],
                                            sensor_pos[1] + rotated_offset[1],
                                            sensor_pos[2]               // z already offset upward
                                        };

                                        // Perform ray cast
                                        double dist = mj_ray(this->model_, this->data_, ray_origin.data(), direction.data(), geom_group, 1, -1, geom_id);

                                        // Compute hit point
                                        std::array<double, 3> hit_point = {
                                            ray_origin[0] + direction[0] * dist,
                                            ray_origin[1] + direction[1] * dist,
                                            ray_origin[2] + direction[2] * dist
                                        };
                                        geometry_msgs::msg::Point ros_pt;
                                        // Making it relative to the base link
                                        ros_pt.x = hit_point[0] - site_pos_global[0];
                                        ros_pt.y = hit_point[1] - site_pos_global[1];
                                        ros_pt.z = hit_point[2];
                                        msg.cells[ii] = ros_pt;
                                        ii++;
                                    }
                                }

                                if (this->model_->sensor_refid[sensor_id] == -1) {
                                    msg.header.frame_id = "world";
                                } else {
                                    int ref_type = this->model_->sensor_reftype[sensor_id];
                                    int ref_id   = this->model_->sensor_refid[sensor_id];
                                    if (ref_type == mjOBJ_SITE) {
                                        msg.header.frame_id =
                                            std::string(this->model_->names + this->model_->name_siteadr[ref_id]);
                                    } else if (ref_type == mjOBJ_BODY) {
                                        msg.header.frame_id =
                                            std::string(this->model_->names + this->model_->name_bodyadr[ref_id]);
                                    } else if (ref_type == mjOBJ_GEOM) {
                                        msg.header.frame_id =
                                            std::string(this->model_->names + this->model_->name_geomadr[ref_id]);
                                    } else if (ref_type == mjOBJ_CAMERA) {
                                        msg.header.frame_id =
                                            std::string(this->model_->names + this->model_->name_camadr[ref_id]);
                                    } else {
                                        RCLCPP_ERROR_STREAM(this->get_logger(),
                                                            "Framepos sensor, "
                                                                << sensor_names.at(i)
                                                                << ", is not associated with a supported Mujoco object "
                                                                   "type! Current object type (mjtObj): "
                                                                << ref_type);
                                    }
                                }
                                has_framepos = true;
                            } else {
                                RCLCPP_ERROR_STREAM(this->get_logger(),
                                                    "There are two framepos associated with this FramePose! Ignoring "
                                                        << sensor_names.at(i));
                            }
                        } else {
                            RCLCPP_ERROR_STREAM(
                                this->get_logger(),
                                "Sensor " << sensor_names.at(i)
                                          << " is not associated with a valid Mujoco sensor type! Current sensor type: "
                                          << mj_sensor_types.at(i));
                        }

                        msg.header.stamp = this->now();
                    }
                    publisher->publish(msg);
                };

                RCLCPP_INFO_STREAM(this->get_logger(), "Timer callback created for a Scan Dots (GridCells) sensor!");
                return cb;
            }
        }

        /**
         * @brief Get the time from the simulation in a format that matches the ROS 2 Time message.
         *
         * Note that sec is relative to the start of the simulation.
         *
         * @warning This function does not apply a Mutex to the data and should only used inside a safe context.
         *
         * @param sec the seconds of the simulation
         * @param nanosec the nanoseconds in the given second
         */
        void GetTimeFromSim(int& sec, uint32_t& nanosec) {
            sec     = std::floor(this->data_->time);
            nanosec = (this->data_->time - sec) * 1e9;
        }

        // -------------------------------------- //
        // ----------- GLFW Callbacks ----------- //
        // -------------------------------------- //
        static void KeyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods) {
            if (mujoco_sim_instance_) {
                mujoco_sim_instance_->HandleKeyboard(window, key, scancode, act, mods);
            } else {
                throw std::runtime_error("No active mujoco sim instance while the GLFW window is running!");
            }
        }

        void HandleKeyboard(__attribute__((unused)) GLFWwindow* window, int key, __attribute__((unused)) int scancode,
                            int act, __attribute__((unused)) int mods) {
            // backspace: reset simulation
            if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
                mj_resetData(model_, data_);
                mj_forward(model_, data_);
            }

            if (act == GLFW_PRESS && key == GLFW_KEY_SPACE) {
                pause = !pause;
            }
        }

        static void MouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
            if (mujoco_sim_instance_) {
                mujoco_sim_instance_->HandleMouseButton(window, button, act, mods);
            } else {
                throw std::runtime_error("No active mujoco sim instance while the GLFW window is running!");
            }
        }

        void HandleMouseButton(GLFWwindow* window, __attribute__((unused)) int button, __attribute__((unused)) int act,
                               __attribute__((unused)) int mods) {
            // update button state
            button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
            button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
            button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

            // update mouse position
            glfwGetCursorPos(window, &lastx, &lasty);
        }

        static void MouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
            if (mujoco_sim_instance_) {
                mujoco_sim_instance_->HandleMouseMove(window, xpos, ypos);
            } else {
                throw std::runtime_error("No active mujoco sim instance while the GLFW window is running!");
            }
        }
        void HandleMouseMove(GLFWwindow* window, double xpos, double ypos) {
            // no buttons down: nothing to do
            if (!button_left && !button_middle && !button_right) {
                return;
            }

            // compute mouse displacement, save
            double dx = xpos - lastx;
            double dy = ypos - lasty;
            lastx     = xpos;
            lasty     = ypos;

            // get current window size
            int width, height;
            glfwGetWindowSize(window, &width, &height);

            // get shift key state
            bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                              glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

            // determine action based on mouse button
            mjtMouse action;
            if (button_right) {
                action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
            } else if (button_left) {
                action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
            } else {
                action = mjMOUSE_ZOOM;
            }

            // move camera
            mjv_moveCamera(model_, action, dx / height, dy / height, &scn, &cam);
        }

        static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
            if (mujoco_sim_instance_) {
                mujoco_sim_instance_->HandleScroll(window, xoffset, yoffset);
            } else {
                throw std::runtime_error("No active mujoco sim instance while the GLFW window is running!");
            }
        }

        void HandleScroll(__attribute__((unused)) GLFWwindow* window, __attribute__((unused)) double xoffset,
                          double yoffset) {
            // emulate vertical mouse motion = 5% of window height
            mjv_moveCamera(model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
        }

        static ObeliskMujocoRobot<ControlMessageT>* mujoco_sim_instance_;

        // MuJoCo data structures
        mjModel* model_ = NULL; // MuJoCo model
        mjData* data_   = NULL; // MuJoCo data
        mjvCamera cam;          // abstract camera
        mjvOption opt;          // visualization options
        mjvScene scn;           // abstract scene
        mjrContext con;         // custom GPU context

        GLFWwindow* window_;

        // mouse interaction
        bool button_left   = false;
        bool button_middle = false;
        bool button_right  = false;
        double lastx       = 0;
        double lasty       = 0;

        bool pause = false;

        // Other memeber variables
        int nu_;
        std::filesystem::path xml_path_;
        float time_step_;
        int num_steps_per_viz_;

        // Key for viz publishing
        std::string viz_key_;

        // Geom id's for viz
        std::vector<int> viz_geoms_;

        // Shared data between the main thread and the sim thread
        std::vector<double> shared_data_;

        // Mutex to manage access to the shared data
        std::mutex shared_data_mut_;
        std::mutex sensor_data_mut_;

        std::atomic<bool> configuration_complete_;
        std::atomic<bool> activation_complete_;
        std::atomic<bool> mujoco_setup_;

        rclcpp::CallbackGroup::SharedPtr callback_group_;

        std::thread rendering_thread_;

        int num_sensors_;

        // For the height map
        std::vector<double> height_map_grid_size_;
        double height_map_grid_spacing_;
        std::vector<long> height_map_geom_group_;

        // Constants
        static constexpr float TIME_STEP_DEFAULT   = 0.002;
        static constexpr int STEPS_PER_VIZ_DEFAULT = 8;

        static constexpr int WINDOW_WIDTH_DEFAULT  = 1200;
        static constexpr int WINDOW_LENGTH_DEFAULT = 900;
    };

    // Define the static member variable
    template <typename T> ObeliskMujocoRobot<T>* ObeliskMujocoRobot<T>::mujoco_sim_instance_ = nullptr;

} // namespace obelisk

#else
// #error "Mujoco support is disabled. Build with USE_MUJOCO=ON to use this header."
#endif

#endif