#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <msg_conversions.h>
#include <mutex>
#include <obelisk_sensor_msgs/msg/obk_image.hpp>
#include <obelisk_std_msgs/msg/u_int8_multi_array.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <set>
#include <sl/Camera.hpp>
#include <stdexcept>
#include <unsupported/Eigen/CXX11/Tensor>
#include <yaml-cpp/yaml.h>

#include "obelisk_sensor.h"

/**
 * @brief Class for managing ZED2 cameras using the Obelisk framework.
 */
class ObeliskZed2Sensors : public obelisk::ObeliskSensor {

  public:
    /**
     * @brief Construct a new ObeliskZed2Sensors object.
     *
     * @param node_name Name of the ROS2 node.
     */
    ObeliskZed2Sensors(const std::string& node_name) : obelisk::ObeliskSensor(node_name) {
        // Declare parameters
        this->declare_parameter("params_path_pkg", "");
        this->declare_parameter("pub_period", 0.0);

        // Get parameter values
        pub_period_                 = this->get_parameter("pub_period").as_double();
        std::string params_path_pkg = this->get_parameter("params_path_pkg").as_string();
        std::string _params_path    = this->get_parameter("params_path").as_string();

        std::filesystem::path params_path;
        if (_params_path[0] == '/') {
            params_path = _params_path;
        } else if (params_path_pkg.empty()) {
            std::string err_msg = "No package provided for ZED2 camera parameters!";
            RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
            throw std::runtime_error(err_msg);
        } else {
            std::string share_directory = ament_index_cpp::get_package_share_directory(params_path_pkg);
            params_path                 = share_directory + "/" + _params_path;
        }

        this->set_camera_params(params_path);

        // Register publisher for ZED2 cameras
        this->RegisterObkPublisher<obelisk_sensor_msgs::msg::ObkImage>("pub_img_setting", pub_img_key_);
    }

    /**
     * @brief Callback for configuring the node.
     *
     * @param prev_state Previous state of the lifecycle node.
     * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
     * Callback return indicating success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& prev_state) {
        obelisk::ObeliskSensor::on_configure(prev_state);

        // Initialize all cameras specified in the configuration yaml
        for (const auto& cam_param : cam_param_dicts_) {
            int cam_index              = cam_param.first;
            const auto& cam_param_dict = cam_param.second;

            // Setting up camera parameters
            sl::InitParameters init_params;
            init_params.camera_resolution      = resolution_;
            init_params.camera_fps             = fps_;
            init_params.depth_mode             = depth_ ? sl::DEPTH_MODE::ULTRA : sl::DEPTH_MODE::NONE;
            init_params.coordinate_units       = sl::UNIT::METER;
            init_params.depth_minimum_distance = 0.3;
            init_params.camera_image_flip      = sl::FLIP_MODE::OFF;

            // Set serial number
            try {
                if (cam_param_dict.find("serial_number") == cam_param_dict.end()) {
                    RCLCPP_ERROR(this->get_logger(), "Serial number key not found for camera %d", cam_index);
                    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
                }

                std::string serial_number_str = cam_param_dict.at("serial_number");
                RCLCPP_INFO(this->get_logger(), "Processing serial number for camera %d: %s", cam_index,
                            serial_number_str.c_str());
                init_params.input.setFromSerialNumber(std::stoi(serial_number_str));
            } catch (const std::invalid_argument& e) {
                RCLCPP_ERROR(this->get_logger(), "Invalid serial number for camera %d: %s", cam_index, e.what());
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Unexpected error for camera %d: %s", cam_index, e.what());
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }

            // Open camera
            cams_[cam_index]   = sl::Camera();
            sl::ERROR_CODE err = cams_[cam_index].open(init_params);
            if (err != sl::ERROR_CODE::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera with serial number %s: %s",
                             cam_param_dict.at("serial_number").c_str(), sl::toString(err).c_str());
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }

            RCLCPP_INFO(this->get_logger(), "Camera with serial number %s initialized!",
                        cam_param_dict.at("serial_number").c_str());

            cam_images_[cam_index] = sl::Mat();

            // Set runtime parameters
            sl::RuntimeParameters rtp;
            rtp.enable_depth          = depth_;
            cam_rt_params_[cam_index] = rtp;
        }

        // Create timers for each camera
        cam_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        for (const auto& cam_param : cam_param_dicts_) {
            int cam_index = cam_param.first;

            auto timer_callback = [this, cam_index]() { this->camera_callback(cam_index); };

            double timer_period = 1.0 / static_cast<double>(fps_);
            auto timer = this->create_wall_timer(std::chrono::duration<double>(timer_period), timer_callback, cam_cbg_);
            std::string timer_key = "timer_" + std::to_string(cam_index);
            timers_[timer_key]    = timer;
        }

        // Create a timer for the publisher
        auto pub_timer_callback = [this]() { this->publish_images(); };
        if (pub_period_ == 0) {
            pub_period_ = 1.0 / static_cast<double>(fps_);
        }
        auto pub_timer =
            this->create_wall_timer(std::chrono::duration<double>(pub_period_), pub_timer_callback, cam_cbg_);
        std::string pub_timer_key = "timer_pub";
        timers_[pub_timer_key]    = pub_timer;

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Callback for shutting down the node.
     *
     * @param prev_state Previous state of the lifecycle node.
     * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
     * Callback return indicating success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& prev_state) {
        obelisk::ObeliskSensor::on_shutdown(prev_state);
        for (auto& cam : cams_) {
            cam.second.close();
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

  private:
    // Key for the image publisher
    std::string const pub_img_key_ = "pub_img";
    double pub_period_;

    // Callback group shared by all camera callbacks
    rclcpp::CallbackGroup::SharedPtr cam_cbg_;

    // Camera parameter maps
    std::map<int, std::map<std::string, std::string>> cam_param_dicts_;
    std::map<int, sl::Camera> cams_;
    std::map<int, sl::Mat> cam_images_;
    std::map<int, sl::RuntimeParameters> cam_rt_params_;
    std::map<int, Eigen::Tensor<uint8_t, 3>> frames_;

    // Shared camera parameters
    sl::RESOLUTION resolution_;
    int fps_;
    bool depth_;
    int height_;
    int width_;

    // Mutex for camera callback
    std::mutex lock_;

    /**
     * @brief Set camera parameters from a YAML configuration file.
     *
     * @param params_path Path to the YAML configuration file.
     */
    void set_camera_params(const std::filesystem::path& params_path) {
        // Check if the file has a .yaml or .yml extension
        if (params_path.extension() != ".yaml" && params_path.extension() != ".yml") {
            std::string err_msg = "Invalid parameters file: " + params_path.string() +
                                  "! ObeliskZed2Sensors expects a YAML file with extension .yaml or .yml.";
            RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
            throw std::runtime_error(err_msg);
        }

        // Load the YAML file
        YAML::Node config;
        try {
            config = YAML::LoadFile(params_path);
        } catch (const YAML::Exception& e) {
            std::string err_msg =
                "Could not load a configuration file at " + params_path.string() + "! Error: " + e.what();
            RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
            throw std::runtime_error(err_msg);
        }

        // Parse the camera parameters
        for (const auto& cam : config) {
            int cam_index              = cam.first.as<int>();
            const auto& cam_param_dict = cam.second;

            // Check if cam_param_dict is a map
            if (!cam_param_dict.IsMap()) {
                std::string err_msg = "Invalid camera parameters for camera " + std::to_string(cam_index) + " in " +
                                      params_path.string() + "!";
                RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
                throw std::runtime_error(err_msg);
            }

            // Check serial number
            if (!cam_param_dict["serial_number"]) {
                std::string err_msg = "Serial number not provided for camera " + std::to_string(cam_index) + " in " +
                                      params_path.string() + "!";
                RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
                throw std::runtime_error(err_msg);
            }
            std::string serial_number = cam_param_dict["serial_number"].as<std::string>();
            if (serial_number.length() != 8) {
                std::string err_msg = "Invalid serial number for camera " + std::to_string(cam_index) + " in " +
                                      params_path.string() + "! Must be 8 digits.";
                RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
                throw std::runtime_error(err_msg);
            }
            cam_param_dicts_[cam_index]["serial_number"] = serial_number;

            // Check resolution
            std::string resolution =
                cam_param_dict["resolution"] ? cam_param_dict["resolution"].as<std::string>() : "VGA";
            std::transform(resolution.begin(), resolution.end(), resolution.begin(), ::tolower);
            if (resolution == "vga") {
                cam_param_dicts_[cam_index]["resolution"] = "VGA";
                resolution_                               = sl::RESOLUTION::VGA;
                height_                                   = 376;
                width_                                    = 672;
            } else if (resolution == "720") {
                cam_param_dicts_[cam_index]["resolution"] = "720";
                resolution_                               = sl::RESOLUTION::HD720;
                height_                                   = 720;
                width_                                    = 1280;
            } else if (resolution == "1080") {
                cam_param_dicts_[cam_index]["resolution"] = "1080";
                resolution_                               = sl::RESOLUTION::HD1080;
                height_                                   = 1080;
                width_                                    = 1920;
            } else if (resolution == "2k") {
                cam_param_dicts_[cam_index]["resolution"] = "2K";
                resolution_                               = sl::RESOLUTION::HD2K;
                height_                                   = 1242;
                width_                                    = 2208;
            } else {
                std::string err_msg = "Invalid resolution for camera " + std::to_string(cam_index) + " in " +
                                      params_path.string() + "! Must be one of 'VGA', '720', '1080', '2K'.";
                RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
                throw std::runtime_error(err_msg);
            }
            cam_param_dicts_[cam_index]["resolution"] = resolution;

            // Check fps
            int fps = cam_param_dict["fps"] ? cam_param_dict["fps"].as<int>() : 0;
            if (fps == 0) {
                if (resolution == "vga")
                    fps = 100;
                else if (resolution == "720")
                    fps = 60;
                else if (resolution == "1080")
                    fps = 30;
                else if (resolution == "2k")
                    fps = 15;
            }
            if (fps != 15 && fps != 30 && fps != 60 && fps != 100) {
                std::string err_msg = "Invalid fps for camera " + std::to_string(cam_index) + " in " +
                                      params_path.string() + "! Must be one of 15, 30, 60, 100.";
                RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
                throw std::runtime_error(err_msg);
            }
            cam_param_dicts_[cam_index]["fps"] = std::to_string(fps);
            fps_                               = fps;

            // Check depth
            bool depth                           = cam_param_dict["depth"] ? cam_param_dict["depth"].as<bool>() : false;
            cam_param_dicts_[cam_index]["depth"] = depth ? "true" : "false";
            depth_                               = depth;

            // Check side
            std::string side = cam_param_dict["side"] ? cam_param_dict["side"].as<std::string>() : "left";
            std::transform(side.begin(), side.end(), side.begin(), ::tolower);
            if (side != "left" && side != "right") {
                std::string err_msg = "Invalid side for camera " + std::to_string(cam_index) + " in " +
                                      params_path.string() + "! Must be one of 'left', 'right'.";
                RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
                throw std::runtime_error(err_msg);
            }
            cam_param_dicts_[cam_index]["side"] = side;
        }

        // Check that all cameras have the same resolution, fps, and depth setting
        auto check_uniformity = [this](const std::map<int, std::map<std::string, std::string>>& cam_param_dicts,
                                       const std::string& key, const std::string& error_message) {
            std::set<std::string> values;
            for (const auto& cam : cam_param_dicts) {
                values.insert(cam.second.at(key));
            }
            if (values.size() > 1) {
                RCLCPP_ERROR_STREAM(this->get_logger(), error_message);
                throw std::runtime_error(error_message);
            }
        };

        check_uniformity(cam_param_dicts_, "resolution", "All cameras must have the same resolution!");
        check_uniformity(cam_param_dicts_, "fps", "All cameras must have the same fps!");
        check_uniformity(cam_param_dicts_, "depth", "All cameras must have the same depth setting!");

        RCLCPP_INFO(this->get_logger(), "Camera parameters set successfully");
    }

    /**
     * @brief Timer callback function to capture images from the camera. Gets registered for a specific camera.
     *
     * @param cam_index Index of the camera.
     */
    void camera_callback(int cam_index) {
        sl::Camera& cam             = cams_[cam_index];
        sl::RuntimeParameters& rtps = cam_rt_params_[cam_index];
        sl::Mat& cam_image          = cam_images_[cam_index];
        std::string side            = cam_param_dicts_[cam_index]["side"];

        if (cam.grab(rtps) == sl::ERROR_CODE::SUCCESS) {
            if (depth_) {
                if (side == "left") {
                    cam.retrieveImage(cam_image, sl::VIEW::DEPTH);
                } else {
                    cam.retrieveImage(cam_image, sl::VIEW::DEPTH_RIGHT);
                }
            } else {
                if (side == "left") {
                    cam.retrieveImage(cam_image, sl::VIEW::LEFT);
                } else {
                    cam.retrieveImage(cam_image, sl::VIEW::RIGHT);
                }
            }

            // Convert sl::Mat to Eigen::Tensor
            int channels = depth_ ? 1 : 3;
            Eigen::Tensor<uint8_t, 3> tensor(height_, width_, channels);

            // Get the pointer to the BGRA data
            const uint8_t* bgra_data = cam_image.getPtr<sl::uchar1>();

            // Convert BGRA to RGB
            for (int y = 0; y < height_; ++y) {
                for (int x = 0; x < width_; ++x) {
                    // populate row-major values, first all red, then green, then blue
                    for (int c = 0; c < channels; ++c) {
                        tensor(y, x, c) = bgra_data[(y * width_ + x) * 4 + (2 - c)];
                    }
                }
            }

            // populate frames_
            {
                std::lock_guard<std::mutex> lock(lock_);
                frames_[cam_index] = std::move(tensor);
            }
        }
    }

    /**
     * @brief Publishes combined images from all cameras as a single message.
     *
     * This function combines images from all connected cameras into a single 4D tensor
     * and publishes it as a ROS message. The tensor has the shape [num_cameras, height, width, channels],
     * where the number of channels is 1 for depth images and 3 for RGB images.
     */
    void publish_images() {
        auto msg = std::make_unique<obelisk_sensor_msgs::msg::ObkImage>();

        // Calculate dimensions
        int num_cameras = cams_.size();
        int channels    = depth_ ? 1 : 3;
        Eigen::Tensor<uint8_t, 4> combined_tensor(num_cameras, height_, width_, channels);

        // make a local version of frames_ to avoid locking the mutex for too long
        std::map<int, Eigen::Tensor<uint8_t, 3>> frames_local;
        {
            std::lock_guard<std::mutex> lock(lock_);
            frames_local = frames_;
        }

        // Read the frames_ map and populate the combined tensor
        int camera_index = 0;
        for (const auto& frame : frames_local) {
            combined_tensor.chip(camera_index, 0) = frame.second;
            camera_index++;
        }

        // Publish the message
        msg->y = obelisk::utils::msgs::TensorToMultiArray(combined_tensor);
        this->GetPublisher<obelisk_sensor_msgs::msg::ObkImage>(pub_img_key_)->publish(std::move(msg));
    }
};
