#pragma once

#include <filesystem>
#include <iostream>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "obelisk_sim_robot.h"

// TODO: Fix issue related to ENV XDG_RUNTIME_DIR=/run/user/1000  # Replace 1000 with your user ID

namespace obelisk {

    template <typename ControlMessageT> class ObeliskMujocoRobot : public ObeliskSimRobot<ControlMessageT> {
      public:
        explicit ObeliskMujocoRobot(const std::string& name) : ObeliskSimRobot<ControlMessageT>(name) {
            this->template declare_parameter<std::string>("mujoco_setting", "");

            mujoco_sim_instance_ = this;
            window_ready_        = false;
        }

        ~ObeliskMujocoRobot() { mujoco_sim_instance_ = nullptr; }

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

            // set configuration paramters

            // setup simulator

            window_ready_ = true;

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

            return on_cleanup(prev_state);
        }

        void ApplyControl(const ControlMessageT& msg) override {}

        // -----------------------------------------//
        // ----------- Simulation Entry ----------- //
        // ---------------------------------------- //
        void RunSimulator() override {
            while (!window_ready_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            while (!this->stop_thread_) {
                if (!glfwWindowShouldClose(window_)) {
                }
            }
        }

      protected:
      private:
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

        std::atomic<bool> window_ready_;

        // Constants
        static constexpr float TIME_STEP_DEFAULT   = 0.002;
        static constexpr int STEPS_PER_VIZ_DEFAULT = 5;

        static constexpr int WINDOW_WIDTH_DEFAULT  = 1200;
        static constexpr int WINDOW_LENGTH_DEFAULT = 900;
    };

    // Define the static member variable
    template <typename T> ObeliskMujocoRobot<T>* ObeliskMujocoRobot<T>::mujoco_sim_instance_ = nullptr;

} // namespace obelisk
