CXX = g++
CXXFLAGS = -Wall -Werror

PORTAUDIO = -I C:/libs/portaudio/include -L C:/libs/portaudio/lib -lportaudio

portaudio_test/portaudio_test.exe: portaudio_test/portaudio_test.cpp
	$(CXX) $^ -o $@ $(CXXFLAGS) $(PORTAUDIO)

clean:
	del portaudio_test\\*.exe recording.raw 2>NUL
