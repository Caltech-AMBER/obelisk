#pragma once

#include <string>
#include <stdexcept>

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
        if (!rclcpp::ok()) { // check if rclpy is not initialized
            rclcpp::init(argc, argv);
        }
        auto node = std::make_shared<NodeT>(name);
        RCLCPP_INFO(node->get_logger(), "Initializing node: %s", name.c_str());
        ExecutorT executor;       
        executor.add_node(node->get_node_base_interface());
        try {
            executor.spin();
        }
        catch (const std::exception& e) {
            // Catch standard exceptions
            RCLCPP_ERROR(node->get_logger(), "Exception during spin: %s", e.what());
        }
        catch (...) {
            // Catch any other unexpected exceptions
            RCLCPP_ERROR(node->get_logger(), "Unknown exception during spin");
        }

        // Cleanup
        executor.remove_node(node->get_node_base_interface());
        node.reset(); // destroy the node

        // Shutdown rclcpp if context is still valid
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
} // namespace obelisk::utils
