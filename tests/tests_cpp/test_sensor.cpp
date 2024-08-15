#include <catch2/catch_test_macros.hpp>

#include "obelisk_sensor.h"

namespace obelisk {
    class ObeliskSensorTester : public ObeliskSensor {
      public:
        ObeliskSensorTester() : ObeliskSensor("obelisk_sensor_tester") {
            this->set_parameter(rclcpp::Parameter("callback_group_settings", ""));
        }

        void Configure() {
            CHECK(this->on_configure(this->get_current_state()) ==
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Activate() {
            CHECK(this->on_activate(this->get_current_state()) ==
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Deactivate() {
            CHECK(this->on_deactivate(this->get_current_state()) ==
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Shutdown() {
            CHECK(this->on_shutdown(this->get_current_state()) ==
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Cleanup() {
            CHECK(this->on_cleanup(this->get_current_state()) ==
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
