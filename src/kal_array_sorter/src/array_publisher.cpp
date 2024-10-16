#include "std_msgs/msg/float64_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <random>

using std::placeholders::_1;

class ArrayPublisher : public rclcpp::Node
{
public:
  ArrayPublisher() : Node("array_publisher")
  {
    array_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("unsorted_array", 1);
    rng.seed(std::random_device{}());
  }

  void publish_random_array()
  {
    std::vector<double> data;
    for (size_t i = 0; i < 7; ++i)
    {
      data.push_back(distribution(rng));
    }

    unsorted_array.data = data;
    array_pub->publish(unsorted_array);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr array_pub;
  std_msgs::msg::Float64MultiArray unsorted_array;

private:
  std::default_random_engine rng;
  std::uniform_real_distribution<double> distribution{0, 1000.0}; //Random számok határértékei
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ArrayPublisher>();

  rclcpp::Rate rate(0.5); //Generálási idő: 0.5Hz = 2mp

  while (rclcpp::ok())
  {
    node->publish_random_array();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
