#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace obelisk::utils {
    template <typename NodeT, typename ExecutorT> void SpinObelisk(int argc, char* argv[], const std::string& name) {
        rclcpp::init(argc, argv);

        auto node = std::make_shared<NodeT>(name);

        ExecutorT executor;
        executor.add_node(node->get_node_base_interface());
        executor.spin();
        executor.remove_node(node->get_node_base_interface());
        rclcpp::shutdown();
    }
} // namespace obelisk::utils
