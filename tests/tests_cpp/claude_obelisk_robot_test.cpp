#include "obelisk_control_msgs/msg/position_setpoint.hpp"
#include "obelisk_robot.h"
#include <catch2/catch_test_macros.hpp>

class TestObeliskRobot : public obelisk::ObeliskRobot<obelisk_control_msgs::msg::PositionSetpoint> {
  public:
    TestObeliskRobot() : ObeliskRobot("test_robot") {}

    void ApplyControl(const obelisk_control_msgs::msg::PositionSetpoint& msg) override {}
};

TEST_CASE("ObeliskRobot Construction and Configuration", "[ObeliskRobot]") {
    rclcpp::init(0, nullptr);

    TestObeliskRobot robot;

    robot.set_parameter(rclcpp::Parameter("sub_ctrl_setting", "topic:/control,history_depth:10"));

    auto result = robot.on_configure(rclcpp_lifecycle::State());
    CHECK(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskRobot Configuration with Empty Sensor Settings", "[ObeliskRobot]") {
    rclcpp::init(0, nullptr);

    TestObeliskRobot robot;

    robot.set_parameter(rclcpp::Parameter("sub_ctrl_setting", "topic:/control,history_depth:10"));

    auto result = robot.on_configure(rclcpp_lifecycle::State());
    CHECK(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskRobot Cleanup and Shutdown", "[ObeliskRobot]") {
    rclcpp::init(0, nullptr);

    TestObeliskRobot robot;
    robot.set_parameter(rclcpp::Parameter("sub_ctrl_setting", "topic:/control,history_depth:10"));
    robot.on_configure(rclcpp_lifecycle::State());

    auto cleanup_result = robot.on_cleanup(rclcpp_lifecycle::State());
    CHECK(cleanup_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    auto shutdown_result = robot.on_shutdown(rclcpp_lifecycle::State());
    CHECK(shutdown_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    rclcpp::shutdown();
}
