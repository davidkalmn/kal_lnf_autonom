#include "std_msgs/msg/float64_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <algorithm>

class ArraySorter : public rclcpp::Node
{
public:
  ArraySorter() : Node("array_sorter")
  {
    array_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "unsorted_array", 10, std::bind(&ArraySorter::process_unsorted_array, this, std::placeholders::_1));

    array_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("sorted_array", 1);
  }

private:
  void process_unsorted_array(const std_msgs::msg::Float64MultiArray::SharedPtr unsorted_array_msg)
  {
    std::vector<double> array_data = unsorted_array_msg->data;

    sort_array(array_data);

    publish_sorted_array(array_data);
  }

  void sort_array(std::vector<double>& data)
  {
    std::sort(data.begin(), data.end());
  }

  void publish_sorted_array(const std::vector<double>& sorted_data)
  {
    std_msgs::msg::Float64MultiArray sorted_array;
    sorted_array.data = sorted_data;
    this->array_pub->publish(sorted_array);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr array_pub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr array_sub;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArraySorter>();

  rclcpp::Rate rate(10);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
