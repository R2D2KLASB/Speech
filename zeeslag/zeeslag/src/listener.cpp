#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "led-matrix.h"
#include "signal.h"

using std::placeholders::_1;
using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

class PublishingSubscriber: public rclcpp::Node {
  public:
    PublishingSubscriber(): Node("publishing_subscriber") {
      subscription_ = this->create_subscription<std_msgs::msg::String>("sub", 10, std::bind(&PublishingSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::String>("pub", 10);

      RGBMatrix::Options my_defaults;
      my_defaults.hardware_mapping = "regular";
      my_defaults.chain_length = 1;
      my_defaults.rows = 16;
      my_defaults.cols = 32;
      my_defaults.show_refresh_rate = false;
      my_defaults.disable_hardware_pulsing = 1;
      my_defaults.parallel = 1;
      rgb_matrix::RuntimeOptions runtime_defaults;
      runtime_defaults.gpio_slowdown = 4;

      canvas = RGBMatrix::CreateFromOptions(my_defaults, runtime_defaults);
      canvas->Fill(255,255,255);
    }
    ~PublishingSubscriber() {
      delete canvas;
    }
  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
      auto message = std_msgs::msg::String();
      message.data = "I heard " + msg->data;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
      canvas->Fill(0,0,0);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    Canvas* canvas;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishingSubscriber>());
  rclcpp::shutdown();
  return 0;
}
