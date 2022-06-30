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

//#include "tiny_gpio.h"
#include "transcription_api.hpp"
#include "pa_recorder.hpp"
#include "string_to_gcode.hpp"
#include <string>
#include <list>      
#include <iostream>     
#include <sstream> 
#include <map> 

class MinimalPublisher : public rclcpp::Node {
public:
	MinimalPublisher() : Node("minimal_publisher") {
		auto message = std_msgs::msg::String();
		//int button = 18;
		//int led = 23;

		publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
		token.getToken();

		//gpioInitialise();
		//gpioSetMode(button, PI_INPUT);
		//gpioSetMode(led, PI_OUTPUT);


		std::string transcription;

		std::map<std::string, std::string> map1_2 = { {"één","1"},
													  {"├®├®n","1"},
													  {"een","1"},
													  {"twee","2"}
		};

		std::string allowed_numbers[14] = { "één","├®├®n","een","twee","1","2","3","4","5","6","7","8","9","10" };
		std::string allowed_letters[10] = { "A","B","C","D","E","F","G","H","I","J" };

		while (true) {
			//if (gpioRead(button)) {
			RCLCPP_INFO(get_logger(), "Recording started");
			//gpioWrite(led, 1);
			rec.record();
			//gpioWrite(led, 0);
			RCLCPP_INFO(get_logger(), "Recording finished");
			transcription = token.transcribeAudio(rec.buffer, rec.size);
			if (transcription != "Transcription failed") {
				std::stringstream ss;
				std::list<std::string> words_temp = {};

				std::string number_move = "";
				std::string number_move_temp = "";
				std::string letter_move = "";
				std::string letter_number_move = "";
				std::string split_character = " ";
				std::string transcription_temp = transcription + " ";
				int index = 0;
				std::string split;
				while ((index = transcription_temp.find(split_character)) != std::string::npos) {
					split = transcription_temp.substr(0, index);
					for (int i = 0; i < split.length(); i++) {
						if (split[i] == ' ' || split[i] == '?' || split[i] == ',' || split[i] == '.') {
							split.erase(i);
						}
					}
					words_temp.push_back(split);
					transcription_temp.erase(0, index + split_character.length());
				}
				for (auto i : words_temp) {
					for (auto j : allowed_letters) {
						if (i == j) { letter_move = i; }
					}
					for (auto n : allowed_numbers) {
						if (i == n) { number_move = i; }
					}
				}


				for (auto i : map1_2) {
					//int index_found_nummer = number_move_temp.find(i.first);
					if (i.first == number_move) {
						number_move = i.second;
					}
				}

				letter_number_move = letter_move + number_move;
				if (letter_move == "" || number_move == "") {
					ss << "Try again!\n";
				}
				else { ss << "i found " << letter_number_move; }

				ss << "\n" << "----- " << transcription << " -----";
				transcription = "";
				transcription.append(ss.str());
				RCLCPP_INFO(get_logger(), "Publishing: '%s'", transcription.c_str());
				message.data = stringToGcode(transcription);
				publisher->publish(message);
			}
			else {
				RCLCPP_INFO(get_logger(), "Transcription failed :(");
				//gpioWrite(led, 1); Pa_Sleep(500); gpioWrite(led, 0); Pa_Sleep(500);
				//gpioWrite(led, 1); Pa_Sleep(500); gpioWrite(led, 0); Pa_Sleep(500);
				//gpioWrite(led, 1); Pa_Sleep(500); gpioWrite(led, 0); Pa_Sleep(500);
			}
		}
		Pa_Terminate();
	}
private:
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
	transcriptionAPI token;
	paRecorder rec;
};

int main(int argc, char* argv[]) {
	Pa_Initialize();
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalPublisher>());
	rclcpp::shutdown();
	return 0;
}