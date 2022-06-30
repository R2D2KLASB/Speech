#include <functional>
#include <memory>
#include <string>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define PORT 8080

using std::placeholders::_1;

class MatrixRelay: public rclcpp::Node {
  public:
    MatrixRelay(): Node("matrix") {
      subscription_ = this->create_subscription<std_msgs::msg::String>("game_info/intern/publish", 10, std::bind(&MatrixRelay::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::String>("game_info/extern/B", 10);
      addrlen = sizeof(address);
      int opt = 1;

      if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
      }
      if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
      }
      address.sin_family = AF_INET;
      address.sin_addr.s_addr = INADDR_ANY;
      address.sin_port = htons(PORT);
      if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
      }
      if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
      }
      if ((new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
      }
      RCLCPP_INFO(this->get_logger(), "socket connected! listening for messages");
    }
    ~MatrixRelay() {
      close(new_socket);
      shutdown(server_fd, SHUT_RDWR);
    }
  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
      char buffer[1024] = { '\0' };
      strcpy((char*)&buffer, msg->data.c_str());

      int recieved = send(new_socket, msg->data.c_str(), 1024, 0);
      RCLCPP_INFO(this->get_logger(), "recieved: '%s'", msg->data.c_str());

      for(int i = 0; i < recieved; i++) buffer[i] = 0;
      read(new_socket, buffer, 1024);
      RCLCPP_INFO(this->get_logger(), "sent: '%s'", &buffer);

      auto message = std_msgs::msg::String();
      message.data = buffer;
      publisher_->publish(message);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    struct sockaddr_in address;
    int server_fd, addrlen, new_socket;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MatrixRelay>());
  rclcpp::shutdown();
  return 0;
}
