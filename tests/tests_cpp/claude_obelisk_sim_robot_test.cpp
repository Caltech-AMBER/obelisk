#include "obelisk_control_msgs/msg/position_setpoint.hpp"
#include "obelisk_sim_robot.h"
#include <catch2/catch_test_macros.hpp>

class TestObeliskSimRobot : public obelisk::ObeliskSimRobot<obelisk_control_msgs::msg::PositionSetpoint> {
  public:
    TestObeliskSimRobot() : ObeliskSimRobot("test_sim_robot") {}

    obelisk_sensor_msgs::msg::TrueSimState PublishTrueSimState() override {
        return obelisk_sensor_msgs::msg::TrueSimState();
    }

    void RunSimulator() override {
        while (!stop_thread_) {
            // Simulate some work
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void ApplyControl(const obelisk_control_msgs::msg::PositionSetpoint& msg) {}

    using ObeliskSimRobot::stop_thread_;
};

TEST_CASE("ObeliskSimRobot Construction and Configuration", "[ObeliskSimRobot]") {
    rclcpp::init(0, nullptr);

    TestObeliskSimRobot robot;

    robot.set_parameter(rclcpp::Parameter("timer_true_sim_state_setting", "timer_period_sec:0.1"));
    robot.set_parameter(rclcpp::Parameter("pub_true_sim_state_setting", "topic:/true_state,history_depth:10"));
    robot.set_parameter(rclcpp::Parameter("sub_ctrl_setting", "topic:topic5"));

    auto result = robot.on_configure(rclcpp_lifecycle::State());
    CHECK(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    robot.on_cleanup(rclcpp_lifecycle::State());

    rclcpp::shutdown();
}

TEST_CASE("ObeliskSimRobot Cleanup and Shutdown", "[ObeliskSimRobot]") {
    rclcpp::init(0, nullptr);

    TestObeliskSimRobot robot;
    robot.set_parameter(rclcpp::Parameter("timer_true_sim_state_setting", "timer_period_sec:0.1"));
    robot.set_parameter(rclcpp::Parameter("pub_true_sim_state_setting", "topic:/true_state,history_depth:10"));
    robot.set_parameter(rclcpp::Parameter("sub_ctrl_setting", "topic:topic5"));
    robot.on_configure(rclcpp_lifecycle::State());

    auto cleanup_result = robot.on_cleanup(rclcpp_lifecycle::State());
    CHECK(cleanup_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    CHECK(robot.stop_thread_ == true);

    auto shutdown_result = robot.on_shutdown(rclcpp_lifecycle::State());
    CHECK(shutdown_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    rclcpp::shutdown();
}
