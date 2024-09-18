#include "rclcpp/rclcpp.hpp"
#include "simulator.h"

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Simulator>());
    rclcpp::shutdown();
    return 0;
}