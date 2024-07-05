#include <catch2/catch_test_macros.hpp>

#include "obelisk_sensor.h"

namespace obelisk {
    class ObeliskSensorTester : public ObeliskSensor {
      public:
        ObeliskSensorTester() : ObeliskSensor("obelisk_sensor_tester") {
            // this->set_parameter(
            //     rclcpp::Parameter("pub_sensor_setting", std::vector<std::string>{"topic:topic2", "topic:topic3"}));
            this->set_parameter(rclcpp::Parameter("callback_group_setting", ""));
        }

        void Configure() {
            REQUIRE(this->on_configure(this->get_current_state()) ==
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Activate() {
            REQUIRE(this->on_activate(this->get_current_state()) ==
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Deactivate() {
            REQUIRE(this->on_deactivate(this->get_current_state()) ==
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Shutdown() {
            REQUIRE(this->on_shutdown(this->get_current_state()) ==
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Cleanup() {
            REQUIRE(this->on_cleanup(this->get_current_state()) ==
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

      protected:
    };
} // namespace obelisk

TEST_CASE("Obelisk Sensor Basic Tests", "[obelisk_sensor]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskSensorTester node;
    // TODO (@zolkin): put back
    // node.Configure();
    // node.Activate();
    // node.Deactivate();
    // node.Shutdown();
    // node.Cleanup();

    rclcpp::shutdown();
}
