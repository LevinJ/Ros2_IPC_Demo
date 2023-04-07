#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(const rclcpp::NodeOptions & options)
    : Node("minimal_subscriber", options)
    {
      count_ = 0;
      RCLCPP_INFO(get_logger(), "Subscriber, use_intra_process_comms=%d", options.use_intra_process_comms());
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalSubscriber::timer_callback, this));
    }
    void timer_callback()
    { 
      RCLCPP_INFO(get_logger(), "recv freq = %d", count_);
      count_ = 0;

    }

  private:
    int count_;
    rclcpp::TimerBase::SharedPtr timer_;
    void topic_callback(const sensor_msgs::msg::Image::UniquePtr msg)
    {
      count_++;
      double dt = (this->now() - msg->header.stamp).seconds() * 1e3;
      RCLCPP_INFO(get_logger(), "id = %s, time gap=%f ms, %p", msg->header.frame_id.c_str(), dt, (void *)reinterpret_cast<std::uintptr_t>(msg.get()));
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(MinimalSubscriber)  // NOLINT

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
   rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  rclcpp::spin(std::make_shared<MinimalSubscriber>(options));
  rclcpp::shutdown();
  return 0;
}