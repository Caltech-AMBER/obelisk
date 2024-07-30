#ifndef OBELISK_ZED2_SENSORS_HPP
#define OBELISK_ZED2_SENSORS_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

class ObeliskZed2Sensors : public rclcpp::Node {
  public:
    ObeliskZed2Sensors(const std::string& node_name) : Node(node_name) {
        RCLCPP_INFO(this->get_logger(), "ObeliskZed2Sensors node created");
    }

    void dummy_yaml_action(const std::string& yaml_string) {
        YAML::Node node = YAML::Load(yaml_string);
        if (node["test_key"]) {
            RCLCPP_INFO(this->get_logger(), "YAML test value: %s", node["test_key"].as<std::string>().c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "YAML test key not found");
        }
    }
};

#endif // OBELISK_ZED2_SENSORS_HPP
