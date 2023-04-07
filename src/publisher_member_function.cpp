#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(const rclcpp::NodeOptions & options)
    : Node("minimal_publisher", options), count_(0)
    {
      RCLCPP_INFO(get_logger(), "Publisher, use_intra_process_comms=%d", options.use_intra_process_comms());
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic", 10);
      timer_ = this->create_wall_timer(
      5ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      // auto message = std_msgs::msg::String();
      // message.data = "Hello, world! " + std::to_string(count_++);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      // publisher_->publish(message);
      static int count = 0;
      static cv::Mat frame_(1920,1080,CV_8UC3);
      sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
      msg->header.frame_id = std::to_string(count++);
      msg->header.stamp =  this->now();
      msg->height = frame_.rows;
      msg->width = frame_.cols;
      msg->encoding = "bgr8";
      msg->is_bigendian = false;
      msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);
      msg->data.assign(frame_.datastart, frame_.dataend);
      // RCLCPP_INFO(this->get_logger(), "Publishing:");
      RCLCPP_INFO(get_logger(), "%s, %p", msg->header.frame_id.c_str(), (void *)reinterpret_cast<std::uintptr_t>(msg.get()));
      publisher_->publish(std::move(msg));
       
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    size_t count_;
};

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(MinimalPublisher)  // NOLINT

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
   // Create NodeOptions, turn off IPC
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  rclcpp::spin(std::make_shared<MinimalPublisher>(options));
  rclcpp::shutdown();
  return 0;
}