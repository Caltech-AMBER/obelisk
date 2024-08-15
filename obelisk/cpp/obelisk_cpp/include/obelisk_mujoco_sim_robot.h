#pragma once

#include <cmath>
#include <cstdarg>
#include <filesystem>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "obelisk_sensor_msgs/msg/obk_joint_encoders.hpp"
#include "obelisk_sim_robot.h"

namespace obelisk {
    template <typename ControlMessageT> class ObeliskMujocoRobot : public ObeliskSimRobot<ControlMessageT> {
      public:
        explicit ObeliskMujocoRobot(const std::string& name) : ObeliskSimRobot<ControlMessageT>(name) {
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

            nu_                = GetNumInputs(mujoco_config_map); // Required
            time_step_         = GetTimeSteps(mujoco_config_map);
            num_steps_per_viz_ = GetNumStepsPerViz(mujoco_config_map);

            shared_data_.resize(nu_);

            configuration_complete_ = true;

            ParseSensorString(mujoco_config_map.at("sensor_settings"));

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

            if (!model_) {
                throw std::runtime_error("Could not load Mujoco model from the XML!");
            }

            data_ = mj_makeData(model_);

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

            // create scene and context
            mjv_makeScene(model_, &scn, 2000);
            mjr_makeContext(model_, &con, mjFONTSCALE_150);

            // install GLFW mouse and keyboard callbacks
            glfwSetKeyCallback(window_, ObeliskMujocoRobot::KeyboardCallback);
            glfwSetCursorPosCallback(window_, ObeliskMujocoRobot::MouseMoveCallback);
            glfwSetMouseButtonCallback(window_, ObeliskMujocoRobot::MouseButtonCallback);
            glfwSetScrollCallback(window_, ObeliskMujocoRobot::ScrollCallback);

            mujoco_setup_ = true;

            while (!activation_complete_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Starting Mujoco simulation loop.");

            // Set the initial condition based on a keyframe
            RCLCPP_INFO_STREAM(this->get_logger(), "Found " << this->model_->nkey << " Mujoco keyframes.");
            for (int i = 0; i < this->model_->nkey; i++) {
                std::string potential_keyframe(this->model_->names + this->model_->name_keyadr[i]);
                if (potential_keyframe == this->get_parameter("ic_keyframe").as_string()) {
                    RCLCPP_INFO_STREAM(this->get_logger(),
                                       "Setting initial condition to keyframe: " << potential_keyframe);
                    mju_copy(this->data_->qpos, &this->model_->key_qpos[i * this->model_->nq], this->model_->nq);
                    mju_copy(this->data_->qvel, &this->model_->key_qvel[i * this->model_->nv], this->model_->nv);
                    this->data_->time = this->model_->key_time[i];

                    mju_copy(this->data_->ctrl, &this->model_->key_ctrl[i * this->model_->nu], this->model_->nu);
                }
            }

            while (!this->stop_thread_) {
                if (!glfwWindowShouldClose(window_)) {
                    auto start_time = std::chrono::steady_clock::now();

                    mjtNum simstart = data_->time;
                    while (data_->time - simstart < num_steps_per_viz_ * time_step_) {
                        {
                            // Get the data from the shared vector safely
                            std::lock_guard<std::mutex> lock(shared_data_mut_);
                            for (int i = 0; i < nu_; i++) {
                                data_->ctrl[i] = shared_data_.at(i);
                            }
                        }

                        std::lock_guard<std::mutex> lock(sensor_data_mut_);
                        mj_step(model_, data_);
                    }

                    // get framebuffer viewport
                    mjrRect viewport = {0, 0, 0, 0};
                    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

                    // update scene and render
                    mjv_updateScene(model_, data_, &opt, NULL, &cam, mjCAT_ALL, &scn);
                    mjr_render(viewport, &scn, &con);

                    // add time stamp in upper-left corner
                    char stamp[50];
                    sprintf_arr(stamp, "Time = %.3f", data_->time);
                    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, stamp, NULL, &con);

                    // TODO (@zolkin): Add option to save the simulation render

                    // swap OpenGL buffers (blocking call due to v-sync)
                    glfwSwapBuffers(window_);

                    // process pending GUI events, call GLFW callbacks
                    glfwPollEvents();

                    auto end_time   = std::chrono::steady_clock::now();
                    auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
                    const int des_time_us = static_cast<int>((num_steps_per_viz_ * time_step_) * 1e6);
                    if (total_time.count() < des_time_us) {
                        const int time_diff_us = des_time_us - total_time.count();
                        std::this_thread::sleep_for(std::chrono::microseconds(time_diff_us));
                    }
                }
            }

            // free visualization storage
            mjv_freeScene(&scn);
            mjr_freeContext(&con);

            // free MuJoCo model and data
            mj_deleteData(data_);
            mj_deleteModel(model_);
        }

      protected:
        obelisk_sensor_msgs::msg::TrueSimState PublishTrueSimState() override {
            static constexpr int FLOATING_BASE = 7;
            static constexpr int FLOATING_VEL  = 6;

            obelisk_sensor_msgs::msg::TrueSimState msg;

            std::lock_guard<std::mutex> lock(sensor_data_mut_);

            msg.header.frame_id = "world";
            msg.header.stamp    = this->now();

            // TODO: Base link name
            // TODO: Joint names
            // TODO: Handle both floating and fixed base

            // Configuration
            for (int i = 0; i < FLOATING_BASE; i++) {
                msg.q_base.emplace_back(this->data_->qpos[i]);
            }

            for (int i = FLOATING_BASE; i < this->model_->nq; i++) {
                msg.q_joints.emplace_back(this->data_->qpos[i]);
            }

            // Velocity
            for (int i = 0; i < FLOATING_VEL; i++) {
                msg.v_base.emplace_back(this->data_->qvel[i]);
            }

            for (int i = FLOATING_VEL; i < this->model_->nv; i++) {
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

        int GetNumInputs(const std::map<std::string, std::string>& config_map) {
            try {
                int nu = std::stoi(config_map.at("n_u"));
                if (nu <= 0) {
                    throw std::runtime_error("Invalid control input dimension - must be greater 0!");
                }
                return nu;
            } catch (const std::exception& e) {
                throw std::runtime_error("No n_u (number of inputs) was provided!");
            }
        }

        float GetTimeSteps(const std::map<std::string, std::string>& config_map) {
            try {
                float time_step = std::stof(config_map.at("time_step"));
                return time_step;
            } catch (const std::exception& e) {
                return TIME_STEP_DEFAULT;
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
                } else if (sensor_type == obelisk_sensor_msgs::msg::ObkFramePose::MESSAGE_NAME) {
                    // Make a publisher and add it to the list
                    auto pub = ObeliskNode::create_publisher<obelisk_sensor_msgs::msg::ObkFramePose>(topic, depth);
                    this->publishers_[sensor_key] =
                        std::make_shared<internal::ObeliskPublisher<obelisk_sensor_msgs::msg::ObkFramePose>>(pub);

                    // Add the timer to the list
                    this->timers_[sensor_key] = this->create_wall_timer(
                        std::chrono::milliseconds(static_cast<uint>(1e3 * dt)),
                        CreateTimerCallback<obelisk_sensor_msgs::msg::ObkFramePose>(
                            sensor_names, mj_sensor_types,
                            this->template GetPublisher<obelisk_sensor_msgs::msg::ObkFramePose>(sensor_key)),
                        callback_group_);
                } else {
                    throw std::runtime_error("Sensor type not supported!");
                }

                this->timers_[sensor_key]->cancel(); // Stop the timer
                num_sensors_++;
            }
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

                        // GetTimeFromSim(msg.stamp.sec, msg.stamp.nanosec);
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
                        // GetTimeFromSim(msg.header.stamp.sec, msg.header.stamp.nanosec);
                    }
                    publisher->publish(msg);
                };

                RCLCPP_INFO_STREAM(this->get_logger(), "Timer callback created for a IMU!");
                return cb;
            } else if constexpr (std::is_same<MessageT, obelisk_sensor_msgs::msg::ObkFramePose>::value) {
                // ------------------------------------------ //
                // ---------- Frame Pose call back ---------- //
                // ------------------------------------------ //
                auto cb = [publisher, sensor_names, mj_sensor_types, this]() {
                    obelisk_sensor_msgs::msg::ObkFramePose msg;

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
                                msg.position.x = this->data_->sensordata[sensor_addr];
                                msg.position.y = this->data_->sensordata[sensor_addr + 1];
                                msg.position.z = this->data_->sensordata[sensor_addr + 2];

                                // Framepos sensors can be mounted to different objects
                                int obj_type = this->model_->sensor_objtype[sensor_id];
                                int obj_id   = this->model_->sensor_objid[sensor_id];
                                if (obj_type == mjOBJ_SITE) {
                                    msg.frame_name =
                                        std::string(this->model_->names + this->model_->name_siteadr[obj_id]);
                                } else if (obj_type == mjOBJ_BODY) {
                                    msg.frame_name =
                                        std::string(this->model_->names + this->model_->name_bodyadr[obj_id]);
                                } else if (obj_type == mjOBJ_GEOM) {
                                    msg.frame_name =
                                        std::string(this->model_->names + this->model_->name_geomadr[obj_id]);
                                } else if (obj_type == mjOBJ_CAMERA) {
                                    msg.frame_name =
                                        std::string(this->model_->names + this->model_->name_camadr[obj_id]);
                                } else {
                                    RCLCPP_ERROR_STREAM(
                                        this->get_logger(),
                                        "Framepos sensor, "
                                            << sensor_names.at(i)
                                            << ", is not associated with a supported Mujoco object type! "
                                               "Current object type (mjtObj): "
                                            << obj_type);
                                }

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
                                msg.orientation.w = this->data_->sensordata[sensor_addr];
                                msg.orientation.x = this->data_->sensordata[sensor_addr + 1];
                                msg.orientation.y = this->data_->sensordata[sensor_addr + 2];
                                msg.orientation.z = this->data_->sensordata[sensor_addr + 3];
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

                        // Note that this might cause weird issues with finite differencing - more testing is needed
                        msg.header.stamp = this->now();
                        // GetTimeFromSim(msg.header.stamp.sec, msg.header.stamp.nanosec);
                    }
                    publisher->publish(msg);
                };

                RCLCPP_INFO_STREAM(this->get_logger(), "Timer callback created for a FramePose!");
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

        // Shared data between the main thread and the sim thread
        std::vector<double> shared_data_;

        // Mutex to manage access to the shared data
        std::mutex shared_data_mut_;
        std::mutex sensor_data_mut_;

        std::atomic<bool> configuration_complete_;
        std::atomic<bool> activation_complete_;
        std::atomic<bool> mujoco_setup_;

        rclcpp::CallbackGroup::SharedPtr callback_group_;

        int num_sensors_;

        // Constants
        static constexpr float TIME_STEP_DEFAULT   = 0.002;
        static constexpr int STEPS_PER_VIZ_DEFAULT = 8;

        static constexpr int WINDOW_WIDTH_DEFAULT  = 1200;
        static constexpr int WINDOW_LENGTH_DEFAULT = 900;
    };

    // Define the static member variable
    template <typename T> ObeliskMujocoRobot<T>* ObeliskMujocoRobot<T>::mujoco_sim_instance_ = nullptr;

} // namespace obelisk
