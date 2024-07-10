#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace obelisk::utils {
    /**
     * @brief spins up an ObeliskNode.
     *  Handles initializing rclcpp, creating the executor, adding the node, spinning, removing the node, and shutting
     * down.
     *
     * @param argc the command line argc
     * @param argv the commmand line argv
     * @param name the name to give to the node
     */
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
