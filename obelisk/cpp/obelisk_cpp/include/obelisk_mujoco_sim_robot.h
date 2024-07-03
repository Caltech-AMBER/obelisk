#pragma once

#include <cstdarg>
#include <filesystem>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "obelisk_sensor_msgs/msg/joint_encoders.hpp"
#include "obelisk_sim_robot.h"

// TODO: Fix issue related to ENV XDG_RUNTIME_DIR=/run/user/1000  # Replace 1000 with your user ID

namespace obelisk {
    template <typename ControlMessageT> class ObeliskMujocoRobot : public ObeliskSimRobot<ControlMessageT> {
      public:
        explicit ObeliskMujocoRobot(const std::string& name) : ObeliskSimRobot<ControlMessageT>(name) {
            this->template declare_parameter<std::string>("mujoco_setting", "");

            mujoco_sim_instance_    = this;
            configuration_complete_ = false;
        }

        ~ObeliskMujocoRobot() { mujoco_sim_instance_ = nullptr; }

        // TODO: Should be we bring up the simulation in configure or activate?

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
            xml_path_ = GetXMLPath(mujoco_config_map); // Required
            // Search for the model
            if (!std::filesystem::exists(xml_path_)) {
                if (const char* obelisk_root = std::getenv("OBELISK_ROOT")) {
                    xml_path_ = static_cast<std::string>(obelisk_root) / xml_path_;
                } else {
                    throw std::runtime_error("OBELISK_ROOT environment variable not set. Run the dev_setup.sh script!");
                }
            }

            nu_                = GetNumInputs(mujoco_config_map); // Required
            time_step_         = GetTimeSteps(mujoco_config_map);
            num_steps_per_viz_ = GetNumStepsPerViz(mujoco_config_map);

            shared_data_.resize(nu_);

            // Setup the sensors
            try {
                ParseSensorString(mujoco_config_map.at("sensor_settings"));
            } catch (const std::exception& e) {
                RCLCPP_WARN_STREAM(this->get_logger(), "Mujoco simulation initialized without any sensors.");
            }

            configuration_complete_ = true;

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
            configuration_complete_ = false;

            // Reset data
            nu_ = -1;

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
            configuration_complete_ = false;

            // Reset data
            nu_ = -1;

            return on_cleanup(prev_state);
        }

        /**
         * @brief Apply the control message.
         * We assume that the control message is a vector of control inputs and is fully compatible with the data.ctrl
         * field of a mujoco model. YOU MUST CHECK THIS YOURSELF!
         */
        void ApplyControl(const ControlMessageT& msg) override { SetSharedData(msg.u); }

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

            RCLCPP_INFO_STREAM(this->get_logger(), "Starting Mujoco simulation loop.");

            while (!this->stop_thread_) {
                if (!glfwWindowShouldClose(window_)) {
                    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
                    //  this loop will finish on time for the next frame to be rendered at 60 fps.
                    //  Otherwise add a cpu timer and exit this loop when it is time to render.
                    mjtNum simstart = data_->time;
                    while (data_->time - simstart < num_steps_per_viz_ * time_step_) {
                        {
                            // Get the data from the shared vector safely
                            std::lock_guard<std::mutex> lock(shared_data_mut_);
                            for (int i = 0; i < nu_; i++) {
                                data_->ctrl[i] = shared_data_.at(i);
                            }
                        }

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
                }
            }

            // TODO: Put this here or in cleanup/shutdown?
            // free visualization storage
            mjv_freeScene(&scn);
            mjr_freeContext(&con);

            // free MuJoCo model and data
            mj_deleteData(data_);
            mj_deleteModel(model_);
        }

      protected:
        obelisk_sensor_msgs::msg::TrueSimState PublishTrueSimState() override {
            obelisk_sensor_msgs::msg::TrueSimState msg;
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
                    sensor_type = setting_map.at("sensor_type");
                } catch (const std::exception& e) {
                    throw std::runtime_error("No sensor type provided for a sensor!");
                }

                std::vector<std::string> sensor_names;
                const std::string name_delim = "&";
                try {
                    std::string names = setting_map.at("sensor_names");
                    while (!names.empty()) {
                        size_t plus_idx = names.find(name_delim);
                        if (plus_idx == std::string::npos) {
                            plus_idx = names.length();
                        }
                        sensor_names.emplace_back(names.substr(0, plus_idx));
                        names.erase(0, plus_idx + name_delim.length());
                    }
                } catch (const std::exception& e) {
                    throw std::runtime_error("No sensor name was provided for a sensor group!");
                }

                // Create the timer and publishers with the settings
                if (sensor_type == "jointpos") {
                    encoder_pubs_.emplace_back(
                        ObeliskNode::create_publisher<obelisk_sensor_msgs::msg::JointEncoders>(topic, depth));

                    rclcpp::CallbackGroup::SharedPtr cbg =
                        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
                    timers_.emplace_back(
                        this->create_wall_timer(std::chrono::duration<double, std::milli>(dt),
                                                CreateTimerCallback(sensor_names, encoder_pubs_.back()), cbg));
                } else {
                    throw std::runtime_error("Sensor type not supported!");
                }
            }
        }

        template <typename MessageT>
        std::function<void()>
        CreateTimerCallback(const std::vector<std::string>& sensor_names,
                            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT>>& publisher) {

            // TODO: Consider specilializing the lambda function for the specific message (i.e. packing the semantic
            // message struct)
            auto cb = [&publisher, sensor_names, this]() {
                MessageT msg;
                for (size_t i = 0; i < sensor_names.size(); i++) {
                    int sensor_id = mj_name2id(this->model_, mjOBJ_SENSOR, sensor_names.at(i).c_str());
                    if (sensor_id == -1) {
                        throw std::runtime_error("Sensor not found in Mujoco! Make sure your XML has the sensor.");
                    }

                    int sensor_addr = this->model_->sensor_adr[sensor_id];

                    msg.y[i]        = this->data_->sensordata[sensor_addr];
                }

                publisher->publish(msg);
            };

            return cb;
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

        void HandleKeyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
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

        void HandleMouseButton(GLFWwindow* window, int button, int act, int mods) {
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

        void HandleScroll(GLFWwindow* window, double xoffset, double yoffset) {
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

        bool pause         = false;

        // Other memeber variables
        int nu_;
        std::filesystem::path xml_path_;
        float time_step_;
        int num_steps_per_viz_;

        // Shared data between the main thread and the sim thread
        std::vector<double> shared_data_;

        // Mutex to manage access to the shared data
        std::mutex shared_data_mut_;

        std::atomic<bool> configuration_complete_;

        // Possible publishers
        std::vector<std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<obelisk_sensor_msgs::msg::JointEncoders>>>
            encoder_pubs_;
        std::vector<rclcpp::TimerBase::SharedPtr> timers_;

        // Constants
        static constexpr float TIME_STEP_DEFAULT   = 0.002;
        static constexpr int STEPS_PER_VIZ_DEFAULT = 8;

        static constexpr int WINDOW_WIDTH_DEFAULT  = 1200;
        static constexpr int WINDOW_LENGTH_DEFAULT = 900;
    };

    // Define the static member variable
    template <typename T> ObeliskMujocoRobot<T>* ObeliskMujocoRobot<T>::mujoco_sim_instance_ = nullptr;

} // namespace obelisk

// TODO: Put back
// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     auto robot =
//     std::make_shared<obelisk::ObeliskMujocoRobot<obelisk_control_msgs::msg::PositionSetpoint>>("mujoco_sim");

//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(robot->get_node_base_interface());
//     executor.spin();

//     rclcpp::shutdown();
// }
