#ifndef OBELISK_RVIZ_PLUGIN__CARTESIAN_TRAJ_DISPLAY_HPP_
#define OBELISK_RVIZ_PLUGIN__CARTESIAN_TRAJ_DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_rendering/objects/line.hpp>

#include "obelisk_control_msgs/msg/cartesian_trajectory.hpp"

namespace obelisk::rviz {
    class CartesianTrajectoryDisplay
        : public rviz_common::MessageFilterDisplay<obelisk_control_msgs::msg::CartesianTrajectory> {
        Q_OBJECT

      protected:
        void processMessage(const obelisk_control_msgs::msg::CartesianTrajectory::ConstSharedPtr msg) override;

        void onInitialize() override;

        std::vector<std::unique_ptr<rviz_rendering::Line>> traj_lines_;

        std::unique_ptr<rviz_common::properties::ColorProperty> color_property_;

      private Q_SLOTS:
        void updateStyle();
    };
} // namespace obelisk::rviz

#endif // OBELISK_RVIZ_PLUGIN__CARTESIAN_TRAJ_DISPLAY_HPP_
