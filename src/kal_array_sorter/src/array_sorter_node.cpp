#include "std_msgs/msg/float64_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <algorithm>

using std::placeholders::_1;

class ArraySorter : public rclcpp::Node
{
public:
    ArraySorter() : Node("array_sorter")
    {
        array_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "unsorted_array", 10, 
            std::bind(&ArraySorter::unsorted_array_callback, this, _1));

        array_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("sorted_array", 1);
    }

    void unsorted_array_callback(const std_msgs::msg::Float64MultiArray& unsorted_array)
    {
        sorted_array.data.clear();
        
        sorted_array.data = unsorted_array.data;
        
        std::sort(sorted_array.data.begin(), sorted_array.data.end());

        array_pub->publish(sorted_array);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr array_pub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr array_sub;
    
    std_msgs::msg::Float64MultiArray sorted_array;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ArraySorter>();

    rclcpp::Rate rate(0.5);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
