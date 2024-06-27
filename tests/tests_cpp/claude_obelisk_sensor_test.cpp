#include "obelisk_sensor.h"
#include <catch2/catch_test_macros.hpp>

class TestObeliskSensor : public obelisk::ObeliskSensor {
  public:
    TestObeliskSensor() : ObeliskSensor("test_sensor") {}

    using ObeliskSensor::pub_sensor_config_strs_;
};

TEST_CASE("ObeliskSensor Construction and Configuration", "[ObeliskSensor]") {
    rclcpp::init(0, nullptr);

    TestObeliskSensor sensor;

    std::vector<std::string> test_configs = {"topic:/sensor1,history_depth:10", "topic:/sensor2,history_depth:20"};
    sensor.set_parameter(rclcpp::Parameter("pub_sensor_settings", test_configs));

    auto result = sensor.on_configure(rclcpp_lifecycle::State());
    REQUIRE(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    REQUIRE(sensor.pub_sensor_config_strs_ == test_configs);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskSensor Configuration with Empty Settings", "[ObeliskSensor]") {
    rclcpp::init(0, nullptr);

    TestObeliskSensor sensor;

    std::vector<std::string> empty_configs = {""};
    sensor.set_parameter(rclcpp::Parameter("pub_sensor_settings", empty_configs));

    REQUIRE_THROWS_AS(sensor.on_configure(rclcpp_lifecycle::State()), std::runtime_error);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskSensor Cleanup and Shutdown", "[ObeliskSensor]") {
    rclcpp::init(0, nullptr);

    TestObeliskSensor sensor;
    std::vector<std::string> test_configs = {"topic:/sensor1,history_depth:10"};
    sensor.set_parameter(rclcpp::Parameter("pub_sensor_settings", test_configs));
    sensor.on_configure(rclcpp_lifecycle::State());

    auto cleanup_result = sensor.on_cleanup(rclcpp_lifecycle::State());
    REQUIRE(cleanup_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    REQUIRE(sensor.pub_sensor_config_strs_.empty());

    auto shutdown_result = sensor.on_shutdown(rclcpp_lifecycle::State());
    REQUIRE(shutdown_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    REQUIRE(sensor.pub_sensor_config_strs_.empty());

    rclcpp::shutdown();
}
