// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <portaudio.h>

#include "transcribe_api.hpp"
#include "pa_recorder.hpp"

class MinimalPublisher: public rclcpp::Node {
public:
  MinimalPublisher(): Node("minimal_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    getToken(&token);

    Pa_Initialize();
    init_recorder(&handle);
    send_transcription();
    send_transcription();
    Pa_Terminate();
  }
private:
  void send_transcription() {
    record(&handle);
    auto message = std_msgs::msg::String();
    message.data = transcribeAudio(&token, handle.buffer, handle.size);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  RECORDER handle;
  TOKEN token;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
