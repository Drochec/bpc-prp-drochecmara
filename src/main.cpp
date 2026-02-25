#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include "RosExampleClass.h"
#include  "helper.hpp"
#include "nodes/io_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create nodes
    //auto node1 = std::make_shared<rclcpp::Node>("node1");
    //auto node2 = std::make_shared<rclcpp::Node>("node2");
    //auto sin_node = std::make_shared<rclcpp::Node>("sin_node");
    auto io_node = std::make_shared<nodes::IoNode>();
    // Regular uptime publishers
    //auto example_class1 = std::make_shared<RosExampleClass>(node1, "topic1", 1.0);
    //auto example_class2 = std::make_shared<RosExampleClass>(node2, "topic2", 2.0);

    // Sine wave publisher
    //auto sin_class = std::make_shared<SinClass>(sin_node, "sin_topic", 50.0);

    // Add nodes to executor (IMPORTANT: add nodes, not classes)
    //executor->add_node(node1);
    //executor->add_node(node2);
    //executor->add_node(sin_node);
    executor->add_node(io_node);

    executor->spin();

    rclcpp::shutdown();
    return 0;
}
