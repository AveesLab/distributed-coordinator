#include "rclcpp/rclcpp.hpp"
#include "coordinator/coordinator.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Coordinator>();
    node->Start();

    //rclcpp::spin(node);
    //rclcpp::shutdown();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    
    return 0;
}

