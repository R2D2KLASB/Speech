/// @file
///
/// \brief
/// coordinates
/// \details
/// 
#ifndef coordinate
#define coordinate

#include <portaudio.h>
#include "transcription_api.hpp"
#include "pa_recorder.hpp"
#include <string>     
#include <iostream>     
#include <map>

/// \brief
/// coordinates_speech function
/// \details
/// This function takes two parameters of type transcriptionAPI and paRecorder, it uses Portaudio library to use the microphone and convert the audio to text using an API.
/// The function returns a string of 2 characters a postion(A5 for example) if the API hears the postion currectly, And "Try again!" if not.
///
std::string Coordinate(transcriptionAPI & token, paRecorder & rec) {
		token.getToken();
		std::string transcription;
		std::string letter_number_move = "";
		std::map<std::string, std::string> map1_2 = { {"één","1"},
													  {"├®├®n","1"},
													  {"een","1"},
													  {"twee","2"},
													  {"10",":"}
		};

		std::string allowed_numbers[14] = { "één","├®├®n","een","twee","1","2","3","4","5","6","7","8","9","10" };
		std::string allowed_letters[10] = { "A","B","C","D","E","F","G","H","I","J" };


			std::cout << "Recording started"<< std::endl;
			rec.record();
			std::cout << "Recording finished"<< std::endl;
			transcription = token.transcribeAudio(rec.buffer, rec.size);
			if (transcription != "Transcription failed") {
				std::list<std::string> words_temp = {};

				std::string number_move = "";
				std::string number_move_temp = "";
				std::string letter_move = "";
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
					if (i.first == number_move) {
						number_move = i.second;
					}
				}

				letter_number_move = letter_move + number_move;
				if (letter_move == "" || number_move == "") {
					return "Try again!";
				}
				else { 
					std::stringstream ss;
					letter_number_move[0] = letter_number_move[0] - 65 + 48;
					letter_number_move[1] = letter_number_move[1] - 1;
					ss << "[" << letter_number_move[0] << "," << letter_number_move[1] << "]";
					return (ss.str());	
				}
			}
			else {
				std::cout << "Transcription fariled :("<< std::endl;
			}
		
		
		return "Try again!"; 
	}


#endif // coordinate
