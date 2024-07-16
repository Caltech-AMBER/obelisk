#include <rviz_common/logging.hpp>

#include <rviz_common/properties/parse_color.hpp>

#include "obelisk_rviz_plugins/cartesian_traj_display.hpp"

namespace obelisk::rviz {
    void CartesianTrajectoryDisplay::processMessage(
        const obelisk_control_msgs::msg::CartesianTrajectory::ConstSharedPtr msg) {
        using rviz_common::properties::StatusProperty;
        RVIZ_COMMON_LOG_INFO_STREAM("We got a message with frame " << msg->header.frame_id);

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
            RVIZ_COMMON_LOG_DEBUG_STREAM("Error transforming from frame '" << msg->header.frame_id << "' to frame '"
                                                                           << qPrintable(fixed_frame_) << "'");
        }

        // Verify the recieved message is self consistent
        if (msg->x.size() != msg->y.size() || msg->y.size() != msg->z.size() || msg->x.size() != msg->z.size()) {
            setStatus(StatusProperty::Warn, "Trajectory length", "x-y-z size mistmatch.");
            return;
        } else {
            setStatus(StatusProperty::Ok, "Trajectory length", "Ok");
        }

        if (msg->x.size() % 2 != 0 || msg->y.size() % 2 != 0 || msg->z.size() % 2 != 0) {
            setStatus(StatusProperty::Warn, "Trajectory points", "Must have an even number of points.");
            return;
        } else {
            setStatus(StatusProperty::Ok, "Trajectory points", "Ok");
        }

        if (traj_lines_.size() < msg->x.size() % 2) {
            for (int i = 0; i < msg->x.size() % 2; i++) {
                // TODO: Do I need unique scene managers and nodes?
                traj_lines_.emplace_back(std::make_unique<rviz_rendering::Line>(scene_manager_, scene_node_));
            }
            updateStyle();
        } else if (traj_lines_.size() > msg->x.size() % 2) {
            traj_lines_.pop_back();
        }

        // for (int line = 0; line < traj_lines_.size() -1; line++) {
        //     Ogre::Vector3 start_point;
        //     Ogre::Vector3 end_point;

        //     start_point.x = msg->x.at(line);
        //     start_point.y = msg->y.at(line);
        //     start_point.z = msg->z.at(line);

        //     end_point.x = msg->x.at(line + 1);
        //     end_point.y = msg->y.at(line + 1);
        //     end_point.z = msg->z.at(line + 1);

        //     traj_lines_.at(line)->setPoints(start_point, end_point);
        // }

        // scene_node_->setPosition(position);
        // scene_node_->setOrientation(orientation);

        RVIZ_COMMON_LOG_INFO_STREAM("message x: " << msg->x.at(0));
        RVIZ_COMMON_LOG_INFO_STREAM("message y: " << msg->y.at(0));
        RVIZ_COMMON_LOG_INFO_STREAM("message z: " << msg->z.at(0));

        // Ogre::Vector3 point_pos;
        // point_pos.x = msg->x.at(0);
        // point_pos.y = msg->y.at(0);
        // point_pos.z = msg->z.at(0);
        // point_shape_->setPosition(point_pos);
    }

    void CartesianTrajectoryDisplay::onInitialize() {
        MFDClass::onInitialize();
        // point_shape_ =
        //     std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_,
        //     scene_node_);

        color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
            "Point Color", QColor(36, 64, 142), "Color to draw the point.", this, SLOT(updateStyle()));

        updateStyle();
    }

    void CartesianTrajectoryDisplay::updateStyle() {
        Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property_->getColor());
        for (auto& lines : traj_lines_) {
            lines->setColor(color);
        }
    }
} // namespace obelisk::rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(obelisk::rviz::CartesianTrajectoryDisplay, rviz_common::Display)
