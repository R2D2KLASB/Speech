#include <string>       
#include <iostream>     
#include <sstream> 
#include <map> 

int main(){
		std::string transcription;

		std::map<std::string, std::string> map1_2 = { {"één","1"},
													  {"├®├®n","1"},
													  {"een","1"},
													  {"twee","2"}
													  //{"2","2"},
													  //{"1","1"},
													  //{"10", "10"}
		};

		std::string allowed_numbers[14] = { "één","├®├®n","een","twee","1","2","3","4","5","6","7","8","9","10" };
		while (true) {
			//if(gpioRead(button)) { 
			//gpioWrite(led, 1);
            std::getline (std::cin,transcription);
			if (transcription != "Transcription failed") {
				std::stringstream ss;
				for(char i : transcription) {
					i = ::tolower(i);
				}
                
                transcription+=".";
                
				std::string find_letter = "letter";
				std::string find_number = "nummer";

				std::string number_move = "";
				std::string number_move_temp = "";
				char letter_move = ' ';
				std::string letter_number_move = "";

				int index_found_letter = transcription.find(find_letter);
				int index_found_number = transcription.find(find_number);


				//  find letter and number
				if (index_found_letter != std::string::npos && index_found_number != std::string::npos) {
					int digits=0;
					int space_index = 0;
					//search "nummer" and check the number after it f map
					for (int i = index_found_number + find_number.length() + 1; i < transcription.length(); i++) {
						if (transcription[i] == ' ' || transcription[i] == '?' || transcription[i] == ',' || transcription[i] == '.') {
							space_index = i;
							break;
						}
					}
					digits = space_index - (index_found_number + find_number.length() + 1);
					number_move_temp = transcription.substr(index_found_number + find_number.length() + 1, digits);
					std::cout <<"number move temp:  "<< number_move_temp << std::endl;
					
					//	for(auto i : allowed_numbers){
					//		if(number_move_temp == i){
					//			
					//		}
					//	}
					
					//look in the map for 1,2 and 10
					for (auto i : map1_2) {
						//int index_found_nummer = number_move_temp.find(i.first);
						if (i.first == number_move_temp) {  //(index_found_nummer != std::string::npos)
							number_move = i.second;
                            std::cout <<"map number move: " << number_move << std::endl;
						}
					}
                    
					// if number wasn't found in map
					if (number_move == "") {
						 int not_digit = number_move_temp.length();
						for (int i = 0; i < number_move_temp.length(); i++) {
							if (number_move_temp[i] > '9' || number_move_temp[i] < '0') {
								not_digit = i;
								break;
							}
						}
                        //it splits the numbers from letters
						number_move = number_move_temp.substr(0, not_digit);

                        std::cout <<"number move: " << number_move << std::endl;
					}
					// find letter
					if (transcription[index_found_letter + find_letter.length()] == 's') {
						letter_move = 'c';
					} else {
						letter_move = transcription[index_found_letter + find_letter.length() + 1];
					}
					
                    
					//add letter and number toegather

					letter_number_move += letter_move + number_move;
                    
                    //////////////////////////////////////////////////////////////////////////////////////////////////////
                    //check of number_move(string) is convertable to number
//                    bool convert = true;
//                    for(char i : number_move){
//                        if (i>'9' || i<'0'){
//                            convert = false;
//                            std::cout << "-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------";
//                        }
//                    }
//                    int NMove=0;
//                    if(convert == true){ NMove = std::stoi(number_move);}// convert string of a number to integer.
                    /////////////////////////////////////////////////////////////////////////////////////////////////////
                    bool out_of_number_range= true;
                    for( std::string i : allowed_numbers){
                        if(number_move == i){
                            out_of_number_range = false;
                        }
                    }
                    
					//check the number and the letter if they were in range.
					if ( out_of_number_range == true || letter_move < 'a' || letter_move > 'j') {
						ss << "Try again!\n";
					}
					else {
						if (letter_number_move != "") { ss << "i found " << letter_number_move; }
						else { ss << "SOMETHING WENT WRONG!\n"; }
					}
				}
				else { //nothing found
					ss << "failed letter or number\n";
				}

				//if game 
				ss << "\n" << "----- " << transcription << " -----" << "\n";
				transcription = "";
				transcription.append(ss.str());
				std::cout << transcription;
			}
			else {
				std::cout << "Transcription failed :( \n";
			}
		}
}



