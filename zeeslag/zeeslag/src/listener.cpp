#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class PublishingSubscriber: public rclcpp::Node {
  public:
    PublishingSubscriber(): Node("publishing_subscriber") {
      subscription_ = this->create_subscription<std_msgs::msg::String>("sub", 10, std::bind(&PublishingSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::String>("pub",10);
    }
  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
      auto message = std_msgs::msg::String();
      message.data = "I heard " + msg->data;
      publisher_->publish(message);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishingSubscriber>());
  rclcpp::shutdown();
  return 0;
}
