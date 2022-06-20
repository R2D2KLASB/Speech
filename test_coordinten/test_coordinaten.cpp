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


class MinimalPublisher : public rclcpp::Node {
public:
	MinimalPublisher() : Node("minimal_publisher") {
		auto message = std_msgs::msg::String();
		std::string transcription;
		//int button = 18;
		//int led = 23;

		publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
		token.getToken();

		//gpioInitialise();
		//gpioSetMode(button, PI_INPUT);
		//    gpioSetMode(led, PI_OUTPUT);
		std::map<std::string, std::string> map1_2 = { {"één","1"},
													  {"├®├®n","1"},
													  {"een","1"},
													  //{"1","1"},
													  {"twee","2"},
													  {"2","2"},
													  {"3","3"},
													  {"10", "10"}
		};
		while (true) {
			//if(gpioRead(button)) { 
			RCLCPP_INFO(get_logger(), "Recording started");
			//gpioWrite(led, 1);
			rec.record();
			//gpioWrite(led, 0);
			RCLCPP_INFO(get_logger(), "Recording finished");
			transcription = token.transcribeAudio(rec.buffer, rec.size);
			if (transcription != "Transcription failed") {
				std::stringstream ss;
				std::for_each(transcription.begin(), transcription.end(), [](char& c) {
					c = ::tolower(c);
					});

				std::string find_letter = "letter";
				std::string find_number = "nummer";

				std::string number_move = "";
				char letter_move = ' ';
				std::string letter_number_move = "";

				int index_found_letter = transcription.find(find_letter);
				int index_found_number = transcription.find(find_number);
				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//look in the map for 1,2 and 10
				for (auto i : map1_2) {
					int index_found_nummer = transcription.find(i.first);
					
					if (index_found_nummer != std::string::npos) {
						number_move = i.second;
						ss << "-------------------- nummer is:  " << number_move << "\n";
					}
				}
				//  find letter and number
				if (index_found_letter != std::string::npos && index_found_number != std::string::npos ) {
					
					// if number wasn't found in map
					if (number_move == "") {
						int digits;
						int space_index = 0;
						for (int i = index_found_number + find_number.length() + 1; i < transcription.length(); i++) {
							if (transcription[i] == ' ' || transcription[i] > '9' || transcription[i] < '0') {
								space_index = i;
								break;
							}
						}
						digits = space_index - (index_found_number + find_number.length() + 1);
						number_move = transcription.substr(index_found_number + find_number.length() + 1 ,digits);
					}

					// find letter
					letter_move = transcription[index_found_letter + find_letter.length() + 1];

					//add letter and number toegather
					letter_number_move = letter_number_move + letter_move + number_move;				
				}
				else { //nothing found
					ss << "failed letter or number";
				}
				
				int NMove = std::stoi(number_move); // convert string of a number to integer.

				//check the number and the letter if they were in range.
				if (NMove > 10 || NMove < 1 || letter_move < 'a' || letter_move > 'j' ) { 
					ss << "Try again!";
				}
				else {
					if(letter_number_move != ""){ss << "i found " << letter_number_move;}
					else { ss << "SOMETHING WENT WRONG!"; }
				}

				////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//if game 
				ss << "\n" << "-----" << transcription << "-----";// << "\n";
				transcription = "";
				transcription.append(ss.str());
				RCLCPP_INFO(get_logger(), "Publishing: '%s'", transcription.c_str());
				message.data = stringToGcode(transcription);
				publisher->publish(message);

				//if printen
				//{
				//RCLCPP_INFO(get_logger(), "Publishing: '%s'", transcription.c_str());
				//	message.data = stringToGcode(transcription);
				//	publisher->publish(message);
				//}	

				
			}
			else {
				RCLCPP_INFO(get_logger(), "Transcription failed :(");
				//gpioWrite(led, 1); Pa_Sleep(500); gpioWrite(led, 0); Pa_Sleep(500);
				//        gpioWrite(led, 1); Pa_Sleep(500); gpioWrite(led, 0); Pa_Sleep(500);
				//        gpioWrite(led, 1); Pa_Sleep(500); gpioWrite(led, 0); Pa_Sleep(500);
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









//if (transcription != "Transcription failed") {
//	RCLCPP_INFO(get_logger(), "Publishing: '%s'", transcription.c_str());
//	message.data = stringToGcode(transcription);
//	publisher->publish(message);
	//std::string find_letter = "Letter";
	//std::string find_number = "nummer";
	//int index_found_letter  = transcription.find(find_letter);
	//int index_found_number = transcription.find(find_number);
	//if (index_found_letter != std::string::npos ) {
	//	char letter_number_move = transcription[index_found_letter + find_letter.length() + 1];
	//	RCLCPP_INFO(get_logger(), "Publishing: '%s'", letter_number_move);
	//}
//}