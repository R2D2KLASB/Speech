#include <portaudio.h>
#include "transcription_api.hpp"
#include "pa_recorder.hpp"
#include "coordinate.hpp"


int main(int argc, char* argv[]) {
	Pa_Initialize();
	transcriptionAPI token;
	paRecorder rec;	
	std::string co = "Try again!";
	while(co =="Try again!"){
		co = Coordinate(token , rec);
		std::cout<< co << std::endl;
	}
	Pa_Terminate();
}





