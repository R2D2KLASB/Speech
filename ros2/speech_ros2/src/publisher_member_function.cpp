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
#include <tiny_gpio.h>

#include "transcription_api.hpp"
#include "pa_recorder.hpp"
#include "string_to_gcode.hpp"

class MinimalPublisher: public rclcpp::Node {
public:
    MinimalPublisher(): Node("minimal_publisher") {
        publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
        token.getToken();
        int button = 18;
        int led = 23;
        gpioSetMode(button, PI_INPUT);
        gpioSetMode(led, PI_OUTPUT);
        for(;;){
            if(gpioRead(button)){
                gpioWrite(led, 1);
                record();
                while(recording()){
                    std::cout << "recording. . .\n";
                    Pa_Sleep(1000);
                }
                gpioWrite(led, 0);
            }
        }
        transcribeAndPublish();
        transcribeAndPublish();
        Pa_Terminate();
    }
private:
    void transcribeAndPublish() {
        auto message = std_msgs::msg::String();
	std::string transcription;

	rec.record();
	transcription = token.transcribeAudio(rec.buffer, rec.size);
	std::cout << transcription << std::endl;

	if(transcription != "Transcription failed") {
	    message.data = stringToGcode(transcription);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher->publish(message);
	}
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    transcriptionAPI token;
    paRecorder rec;
};

int main(int argc, char * argv[]) {
    Pa_Initialize();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
