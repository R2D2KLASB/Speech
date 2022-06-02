# Speech
Voor de codestandaard, klik [hier](https://github.com/R2D2KLASB/Info/blob/main/CodeStandaard.md)

# Table of Contents
1. [Installation](#installation)
2. [Preparation](#preparation)
3. [Build & Run](#build&run)

# Installation <a name="installation"></a>
## Ubuntu 20.04 LTS (RPi4)
1. Everything has been tested on ubuntu focal fossa (20.04) LTS.
2. Follow the installation guide to install [ros2](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Install-Binary.html).
3. Install the required packages for ros2 and portaudio: 
    > sudo apt install python3-colcon-common-extensions libssl-dev alsa-utils libasound-dev
4. Clone the latest portaudio source files in `/home/ubuntu`:
    > git clone https://github.com/PortAudio/portaudio.git
5. Run the following command to generate the build files from `/home/ubuntu/portaudio`:
    > cmake . -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/ubuntu/portaudio/install
6. Now build and install portaudio from `/home/ubuntu/portaudio/install`:
    > make && make install

# Preparation <a name="preparation"></a>
Before you can run the speech node you have to set your mic as the default input device and give yourself permissions to use GPIO since ros should not executed as root.

1. Take note of the **card number** and **device number** of the recording the recording device you want to use:
    > arecord -l
3. Create the default audio device `nano ~/.asoundrc` and insert the config below, replace the card and device number with the results from the previous instruction:

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
4. Give the user access to GPIO (until a workaround is found this has to be executed every boot).
    > sudo chown ubuntu /dev/gpiomem

# Build & Run <a name="build&run"></a>
1. Clone this repository to `/home/ubuntu`.
2. Move to speech_ros2 and source the ros2 environment (run once every terminal instance):
    > . ~/ros2_foxy/ros2-linux/setup.bash
3. Build the speech node: 
    > colcon build --merge-install --packages-select speech_ros2
4. Run the setup files (run once every terminal instance):
    > . install/setup.bash
5. Execute the node:
    > ros2 run speech_ros2 talker
