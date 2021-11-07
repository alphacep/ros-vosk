ros_vosk
======================

A ROS package for speech-to-text services based on [Vosk](https://github.com/alphacep/vosk-api)

## Tutorials

1. Install this package and vosk

  ```bash
  sudo apt install ros-${ROS_DISTRO}-ros-vosk
  ```
  don't forget to run `catkin_make`
  
2. Install Dependencies

  If using ROS MELODIC run first: 
  ```bash
  sudo apt install python3-pip python3-yaml
  ```
  Then run for MELODIC & NOETIC:
  ```bash
  pip3 install sounddevice
  pip3 install vosk
  ``` 
  And if you want to use the TTS engine please run:
  ```bash
  sudo apt install espeak
  pip install pyttsx3
  ```  
3. Launch the node

  Launch the speech recognition node

  ```bash
  roslaunch ros_vosk ros_vosk.launch
  ```
  or by running:
  ```bash
  rosrun ros_vosk vosk_node.py
  ```

## Interface

### Publishing Topics
* speech_recognition/vosk_result    -> vosk_node.py publishes a custom "speech_recognition" message
* speech_recognition/final_result   -> vosk_node.py publishes a simple string with the final result
* speech_recognition/partial_result -> vosk_node.py publishes a simple string with the partial result
* tts/status -> tts_engine.py publishes the state of the engine. True if it is speaking False if it is not. If the status is true vosk_node won't process the audio stream so it won't listen to itself 
* tts/phrase -> tts_engine.py subscribes to this topic in order to speak the given string. Name your desire and it shall be heard by all in the room..

## Author
Angelo Antikatzidis <an.antikatzidis@gmail.com>
Nickolay V. Shmyrev <nshmyrev@gmail.com>
