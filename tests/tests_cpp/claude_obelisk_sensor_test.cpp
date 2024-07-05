#include "obelisk_sensor.h"
#include <catch2/catch_test_macros.hpp>

class TestObeliskSensor : public obelisk::ObeliskSensor {
  public:
    TestObeliskSensor() : ObeliskSensor("test_sensor") {}
};

// TEST_CASE("ObeliskSensor Cleanup and Shutdown", "[ObeliskSensor]") {
//     rclcpp::init(0, nullptr);

//     TestObeliskSensor sensor;
//     // std::vector<std::string> test_configs = {"topic:/sensor1,history_depth:10"};
//     // sensor.set_parameter(rclcpp::Parameter("pub_sensor_settings", test_configs));
//     sensor.on_configure(rclcpp_lifecycle::State());

//     auto cleanup_result = sensor.on_cleanup(rclcpp_lifecycle::State());
//     REQUIRE(cleanup_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

//     auto shutdown_result = sensor.on_shutdown(rclcpp_lifecycle::State());
//     REQUIRE(shutdown_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

//     rclcpp::shutdown();
// }
