#include "obelisk_control_msgs/msg/position_setpoint.hpp"
#include "obelisk_robot.h"
#include <catch2/catch_test_macros.hpp>

class TestObeliskRobot : public obelisk::ObeliskRobot<obelisk_control_msgs::msg::PositionSetpoint> {
  public:
    TestObeliskRobot() : ObeliskRobot("test_robot") {}

    void ApplyControl(const obelisk_control_msgs::msg::PositionSetpoint& msg) override {}

    using ObeliskRobot::control_subscriber_;
    using ObeliskRobot::pub_sensor_config_strs_;
};

TEST_CASE("ObeliskRobot Construction and Configuration", "[ObeliskRobot]") {
    rclcpp::init(0, nullptr);

    TestObeliskRobot robot;

    robot.set_parameter(rclcpp::Parameter("sub_ctrl_setting", "topic:/control,history_depth:10"));
    // std::vector<std::string> test_configs = {"topic:/sensor1,history_depth:10", "topic:/sensor2,history_depth:20"};
    // robot.set_parameter(rclcpp::Parameter("pub_sensor_setting", test_configs));

    auto result = robot.on_configure(rclcpp_lifecycle::State());
    REQUIRE(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    REQUIRE(robot.control_subscriber_ != nullptr);
    // REQUIRE(robot.pub_sensor_config_strs_ == test_configs);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskRobot Configuration with Empty Sensor Settings", "[ObeliskRobot]") {
    rclcpp::init(0, nullptr);

    TestObeliskRobot robot;

    robot.set_parameter(rclcpp::Parameter("sub_ctrl_setting", "topic:/control,history_depth:10"));
    // std::vector<std::string> empty_configs = {""};
    // robot.set_parameter(rclcpp::Parameter("pub_sensor_settings", empty_configs));

    auto result = robot.on_configure(rclcpp_lifecycle::State());
    REQUIRE(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    REQUIRE(robot.control_subscriber_ != nullptr);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskRobot Cleanup and Shutdown", "[ObeliskRobot]") {
    rclcpp::init(0, nullptr);

    TestObeliskRobot robot;
    robot.set_parameter(rclcpp::Parameter("sub_ctrl_setting", "topic:/control,history_depth:10"));
    // std::vector<std::string> test_configs = {"topic:/sensor1,history_depth:10"};
    // robot.set_parameter(rclcpp::Parameter("pub_sensor_settings", test_configs));
    robot.on_configure(rclcpp_lifecycle::State());

    auto cleanup_result = robot.on_cleanup(rclcpp_lifecycle::State());
    REQUIRE(cleanup_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    REQUIRE(robot.control_subscriber_ == nullptr);
    REQUIRE(robot.pub_sensor_config_strs_.empty());

    auto shutdown_result = robot.on_shutdown(rclcpp_lifecycle::State());
    REQUIRE(shutdown_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    REQUIRE(robot.control_subscriber_ == nullptr);
    REQUIRE(robot.pub_sensor_config_strs_.empty());

    rclcpp::shutdown();
}
