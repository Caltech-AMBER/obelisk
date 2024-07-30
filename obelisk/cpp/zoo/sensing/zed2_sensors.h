#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <mutex>
#include <obelisk_sensor_msgs/msg/obk_image.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <set>
#include <sl/Camera.hpp>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "obelisk_sensor.h"

class ObeliskZed2Sensors : public obelisk::ObeliskSensor {

  public:
    ObeliskZed2Sensors(const std::string& node_name) : obelisk::ObeliskSensor(node_name) {
        // Declare parameters
        this->declare_parameter("params_path_pkg", "");
        this->declare_parameter("params_path", "");

        // Get parameter values
        std::string params_path_pkg = this->get_parameter("params_path_pkg").as_string();
        std::string _params_path    = this->get_parameter("params_path").as_string();

        std::string params_path;
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
        pub_img_ = this->create_publisher<obelisk_sensor_msgs::msg::ObkImage>("pub_img", 10);
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& prev_state) {
        obelisk::ObeliskSensor::on_configure(prev_state);

        // Initialize cameras
        for (const auto& cam_param : cam_param_dicts_) {
            int cam_index              = cam_param.first;
            const auto& cam_param_dict = cam_param.second;

            // Initialize camera
            sl::InitParameters init_params;
            init_params.camera_resolution      = resolution_;
            init_params.camera_fps             = fps_;
            init_params.depth_mode             = depth_ ? sl::DEPTH_MODE::ULTRA : sl::DEPTH_MODE::NONE;
            init_params.coordinate_units       = sl::UNIT::METER;
            init_params.depth_minimum_distance = 0.3;
            init_params.camera_image_flip      = sl::FLIP_MODE::OFF;

            // Set serial number
            init_params.input.setFromSerialNumber(std::stoi(cam_param_dict.at("serial_number")));

            // Open camera
            sl::Camera camera;
            sl::ERROR_CODE err = camera.open(init_params);
            if (err != sl::ERROR_CODE::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera with serial number %s: %s",
                             cam_param_dict.at("serial_number").c_str(), sl::toString(err).c_str());
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }

            RCLCPP_INFO(this->get_logger(), "Camera with serial number %s initialized!",
                        cam_param_dict.at("serial_number").c_str());

            cams_[cam_index]       = std::move(camera);
            cam_polled_[cam_index] = false;
            cam_images_[cam_index] = sl::Mat();

            // Set runtime parameters
            sl::RuntimeParameters rtp;
            rtp.enable_depth          = depth_;
            cam_rt_params_[cam_index] = rtp;
        }

        // Create timers for each camera
        for (const auto& cam_param : cam_param_dicts_) {
            int cam_index = cam_param.first;

            auto timer_callback = [this, cam_index]() { this->camera_callback(cam_index); };

            double timer_period = 1.0 / static_cast<double>(fps_);
            auto timer          = this->create_wall_timer(std::chrono::duration<double>(timer_period), timer_callback);
            std::string timer_key = "timer_" + std::to_string(cam_index);
            timers_[timer_key]    = timer;
        }

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

  private:
    // Private member variables
    std::map<int, sl::Camera> cams_;
    std::map<int, bool> cam_polled_;
    std::map<int, sl::Mat> cam_images_;
    std::map<int, sl::RuntimeParameters> cam_rt_params_;
    std::map<int, std::vector<uint8_t>> frames_;
    std::mutex lock_;

    rclcpp::Publisher<obelisk_sensor_msgs::msg::ObkImage>::SharedPtr pub_img_;
    sl::RESOLUTION resolution_;
    int fps_;
    bool depth_;
    std::map<int, std::map<std::string, std::string>> cam_param_dicts_;

    int height_;
    int width_;

    void set_camera_params(const std::string& params_path) {
        // Check if the file has a .yaml or .yml extension
        if (params_path.substr(params_path.find_last_of(".") + 1) != "yaml" &&
            params_path.substr(params_path.find_last_of(".") + 1) != "yml") {
            std::string err_msg = "Invalid parameters file: " + params_path +
                                  "! ObeliskZed2Sensors expects a YAML file with extension .yaml or .yml.";
            RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
            throw std::runtime_error(err_msg);
        }

        // Load the YAML file
        YAML::Node config;
        try {
            config = YAML::LoadFile(params_path);
        } catch (const YAML::Exception& e) {
            std::string err_msg = "Could not load a configuration file at " + params_path + "! Error: " + e.what();
            RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
            throw std::runtime_error(err_msg);
        }

        // Parse the camera parameters
        for (const auto& cam : config) {
            int cam_index              = cam.first.as<int>();
            const auto& cam_param_dict = cam.second;

            // Check if cam_param_dict is a map
            if (!cam_param_dict.IsMap()) {
                std::string err_msg =
                    "Invalid camera parameters for camera " + std::to_string(cam_index) + " in " + params_path + "!";
                RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
                throw std::runtime_error(err_msg);
            }

            // Check serial number
            if (!cam_param_dict["serial_number"]) {
                std::string err_msg =
                    "Serial number not provided for camera " + std::to_string(cam_index) + " in " + params_path + "!";
                RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
                throw std::runtime_error(err_msg);
            }
            std::string serial_number = cam_param_dict["serial_number"].as<std::string>();
            if (serial_number.length() != 8) {
                std::string err_msg = "Invalid serial number for camera " + std::to_string(cam_index) + " in " +
                                      params_path + "! Must be 8 digits.";
                RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
                throw std::runtime_error(err_msg);
            }

            // Check resolution
            std::string resolution =
                cam_param_dict["resolution"] ? cam_param_dict["resolution"].as<std::string>() : "VGA";
            std::transform(resolution.begin(), resolution.end(), resolution.begin(), ::tolower);
            if (resolution == "vga") {
                cam_param_dicts_[cam_index]["resolution"] = "VGA";
                height_                                   = 376;
                width_                                    = 672;
            } else if (resolution == "720") {
                cam_param_dicts_[cam_index]["resolution"] = "HD720";
                height_                                   = 720;
                width_                                    = 1280;
            } else if (resolution == "1080") {
                cam_param_dicts_[cam_index]["resolution"] = "HD1080";
                height_                                   = 1080;
                width_                                    = 1920;
            } else if (resolution == "2k") {
                cam_param_dicts_[cam_index]["resolution"] = "HD2K";
                height_                                   = 1242;
                width_                                    = 2208;
            } else {
                std::string err_msg = "Invalid resolution for camera " + std::to_string(cam_index) + " in " +
                                      params_path + "! Must be one of 'VGA', '720', '1080', '2K'.";
                RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
                throw std::runtime_error(err_msg);
            }

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
                std::string err_msg = "Invalid fps for camera " + std::to_string(cam_index) + " in " + params_path +
                                      "! Must be one of 15, 30, 60, 100.";
                RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
                throw std::runtime_error(err_msg);
            }
            cam_param_dicts_[cam_index]["fps"] = std::to_string(fps);

            // Check depth
            bool depth                           = cam_param_dict["depth"] ? cam_param_dict["depth"].as<bool>() : false;
            cam_param_dicts_[cam_index]["depth"] = depth ? "true" : "false";

            // Check side
            std::string side = cam_param_dict["side"] ? cam_param_dict["side"].as<std::string>() : "left";
            std::transform(side.begin(), side.end(), side.begin(), ::tolower);
            if (side != "left" && side != "right") {
                std::string err_msg = "Invalid side for camera " + std::to_string(cam_index) + " in " + params_path +
                                      "! Must be one of 'left', 'right'.";
                RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
                throw std::runtime_error(err_msg);
            }
            cam_param_dicts_[cam_index]["side"] = side;
        }

        // Check that all cameras have the same resolution, fps, and depth setting
        std::set<std::string> resolutions, fpss, depths;
        for (const auto& cam : cam_param_dicts_) {
            resolutions.insert(cam.second.at("resolution"));
            fpss.insert(cam.second.at("fps"));
            depths.insert(cam.second.at("depth"));
        }

        if (resolutions.size() > 1) {
            throw std::runtime_error("All cameras must have the same resolution!");
        }
        if (fpss.size() > 1) {
            throw std::runtime_error("All cameras must have the same fps!");
        }
        if (depths.size() > 1) {
            throw std::runtime_error("All cameras must have the same depth setting!");
        }

        resolution_ = sl::RESOLUTION::HD720; // Default value, you might want to set this based on the parsed resolution
        fps_        = std::stoi(*fpss.begin());
        depth_      = *depths.begin() == "true";

        RCLCPP_INFO(this->get_logger(), "Camera parameters set successfully");
    }

    void camera_callback(int cam_index) {
        std::lock_guard<std::mutex> lock(lock_);

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

            // Convert sl::Mat to std::vector<uint8_t>
            size_t size = cam_image.getWidth() * cam_image.getHeight() * cam_image.getChannels();
            frames_[cam_index].resize(size);
            memcpy(frames_[cam_index].data(), cam_image.getPtr<sl::uchar1>(), size);

            cam_polled_[cam_index] = true;
            RCLCPP_INFO(this->get_logger(), "Polled camera %d!", cam_index);

            check_and_publish_images();
        }
    }

    void check_and_publish_images() {
        if (std::all_of(cam_polled_.begin(), cam_polled_.end(), [](const auto& pair) { return pair.second; })) {
            auto msg = std::make_unique<obelisk_sensor_msgs::msg::ObkImage>();
            // Prepare the UInt8MultiArray
            msg->y.layout.data_offset = 0;

            // Calculate dimensions
            int num_cameras         = frames_.size();
            int channels            = depth_ ? 1 : 3;
            std::array<int, 4> dims = {num_cameras, height_, width_, channels};

            // Prepare dimension information
            msg->y.layout.dim.resize(4);
            int stride = 1;
            for (int i = 3; i >= 0; --i) {
                auto& dim  = msg->y.layout.dim[i];
                dim.label  = (i == 0) ? "cameras" : (i == 1) ? "height" : (i == 2) ? "width" : "channels";
                dim.size   = dims[i];
                dim.stride = stride;
                stride *= dims[i];
            }

            // Combine images from all cameras into a single vector
            std::vector<uint8_t> combined_data;
            combined_data.reserve(num_cameras * height_ * width_ * channels);

            for (const auto& frame : frames_) {
                combined_data.insert(combined_data.end(), frame.second.begin(), frame.second.end());
            }

            // Assign the combined data to the message
            msg->y.data = std::move(combined_data);

            // Publish the message
            pub_img_->publish(std::move(msg));

            // Reset poll flags
            for (auto& polled : cam_polled_) {
                polled.second = false;
            }
        }
    }
};
