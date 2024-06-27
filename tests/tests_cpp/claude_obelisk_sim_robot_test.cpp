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

    using ObeliskSimRobot::GetSharedData;
    using ObeliskSimRobot::n_u_;
    using ObeliskSimRobot::SetSharedData;
    using ObeliskSimRobot::stop_thread_;
    using ObeliskSimRobot::true_sim_state_publisher_;
    using ObeliskSimRobot::true_sim_state_timer_;
};

TEST_CASE("ObeliskSimRobot Construction and Configuration", "[ObeliskSimRobot]") {
    rclcpp::init(0, nullptr);

    TestObeliskSimRobot robot;

    robot.set_parameter(rclcpp::Parameter("n_u", 3));
    robot.set_parameter(rclcpp::Parameter("timer_true_sim_state_setting", "timer_period_sec:0.1"));
    robot.set_parameter(rclcpp::Parameter("pub_true_sim_state_setting", "topic:/true_state,history_depth:10"));

    auto result = robot.on_configure(rclcpp_lifecycle::State());
    REQUIRE(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    REQUIRE(robot.n_u_ == 3);
    REQUIRE(robot.true_sim_state_publisher_ != nullptr);
    REQUIRE(robot.true_sim_state_timer_ != nullptr);

    robot.on_cleanup(rclcpp_lifecycle::State());

    rclcpp::shutdown();
}

TEST_CASE("ObeliskSimRobot Shared Data", "[ObeliskSimRobot]") {
    rclcpp::init(0, nullptr);

    TestObeliskSimRobot robot;
    robot.set_parameter(rclcpp::Parameter("n_u", 3));
    robot.on_configure(rclcpp_lifecycle::State());

    std::vector<double> test_data = {1.0, 2.0, 3.0};
    robot.SetSharedData(test_data);

    std::vector<double> retrieved_data;
    robot.GetSharedData(retrieved_data);

    REQUIRE(retrieved_data == test_data);

    robot.on_cleanup(rclcpp_lifecycle::State());

    rclcpp::shutdown();
}

TEST_CASE("ObeliskSimRobot Cleanup and Shutdown", "[ObeliskSimRobot]") {
    rclcpp::init(0, nullptr);

    TestObeliskSimRobot robot;
    robot.set_parameter(rclcpp::Parameter("n_u", 3));
    robot.set_parameter(rclcpp::Parameter("timer_true_sim_state_setting", "timer_period_sec:0.1"));
    robot.set_parameter(rclcpp::Parameter("pub_true_sim_state_setting", "topic:/true_state,history_depth:10"));
    robot.on_configure(rclcpp_lifecycle::State());

    auto cleanup_result = robot.on_cleanup(rclcpp_lifecycle::State());
    REQUIRE(cleanup_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    REQUIRE(robot.n_u_ == -1);
    REQUIRE(robot.true_sim_state_publisher_ == nullptr);
    REQUIRE(robot.true_sim_state_timer_ == nullptr);
    REQUIRE(robot.stop_thread_ == true);

    auto shutdown_result = robot.on_shutdown(rclcpp_lifecycle::State());
    REQUIRE(shutdown_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    rclcpp::shutdown();
}
