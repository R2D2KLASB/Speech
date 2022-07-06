# Speech
Voor de codestandaard, klik [hier](https://github.com/R2D2KLASB/Info/blob/main/CodeStandaard.md)

# Table of Contents
1. [Installation](#installation)
2. [Speech to Gcode](#gcode)
    1. [Preparation](#preparation)
    2. [Build & Run](#build&run)
3. [Battleship](#battleship)
    1. [Preparation](#preparation2)
    2. [Build & Run](#build&run2)

<br>

# Installation <a name="installation"></a>
## Ubuntu 20.04 LTS (RPi4)
1. Everything has been tested on ubuntu focal fossa (20.04) LTS.
2. Follow the installation guide to install [ros2](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Install-Binary.html).
3. Clone this repository to `/home/ubuntu`.
4. Clone the led-matrix library to `/home/ubuntu`.
    > git clone https://github.com/hzeller/rpi-rgb-led-matrix
5. Install the required packages for ros2 and portaudio: 
    > sudo apt install python3-colcon-common-extensions libssl-dev alsa-utils libasound-dev
6. Clone the latest portaudio source files in `/home/ubuntu`:
    > git clone https://github.com/PortAudio/portaudio.git
7. Run the following command to generate the build files from `/home/ubuntu/portaudio`:
    > cmake . -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/ubuntu/portaudio/install
8. Now build and install portaudio from `/home/ubuntu/portaudio/install`:
    > make && make install

<br>

# Speech to Gcode <a name="gcode"></a>
# Preparation <a name="preparation"></a>
Before you can run the speech node you have to set your mic as the default input device and give yourself permissions to use GPIO since ros should not executed as root.

1. Take note of the **card number** and **device number** of the recording the recording device you want to use:
    > arecord -l
2. Create the default audio device `nano ~/.asoundrc` and insert the config below, replace the card and device number with the results from the previous instruction:

        pcm.!default {
            type asym
            capture.pcm "mic"
        }
        pcm.mic {
            type plug
            slave {
                pcm "hw:<card number>,<device number>"
            }
        }
3. Give the user access to GPIO (until a workaround is found this has to be executed every boot).
    > sudo chown ubuntu /dev/gpiomem

# Build & Run <a name="build&run"></a>
1. Move to speech_ros2 and source the ros2 environment (run once every terminal instance):
    > . ~/ros2_foxy/ros2-linux/setup.bash
2. Build the speech node: 
    > colcon build --merge-install --packages-select speech_ros2
3. Run the setup files (run once every terminal instance):
    > . install/setup.bash
4. Execute the node:
    > ros2 run speech_ros2 talker

<br>

# Battleship <a name="battleship"></a>

# Preparation <a name="preparation2"></a>
1. Change the defines in animations.hpp corresponding to your matrix setup, do:
    >nano ~/Speech/led-matrix/animations.hpp
       
    ```c++
        #define Rows 16
        #define Cols 32
        #define Parallel 2
    ```
2. Give the user access to GPIO (until a workaround is found this has to be executed every boot).
    > sudo chown ubuntu /dev/gpiomem 

# Build & Run <a name="build&run2"></a>
1. Source the ros2 environment (run once every terminal instance):
    > . ~/ros2_foxy/ros2-linux/setup.bash
2. Move to `~/Speech/zeeslag` and do colcon build:
    > cd  ~/Speech/zeeslag\
    > colcon build
3. Now run the node with: 
    > ros2 run zeeslag matrixrelay
4. Open a second terminal instance.
5. Move to `~/Speech/led-matrix` and do:
    > cmake .\
    > make
6. To run do:
    > ./matrix



![IMG_20220706_190316__01__01](https://user-images.githubusercontent.com/43569137/177605218-ff4a4a69-c67f-48d0-a154-31586f3163ba.jpg)
![IMG_20220706_190807__01](https://user-images.githubusercontent.com/43569137/177606178-cfcc193f-6423-49e8-a6d5-4c33db613449.jpg)

