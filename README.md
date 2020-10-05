ros_vosk
======================

A ROS package for speech-to-text services based on [Vosk](https://gtihub.com/alphacep/vosk-api)

Mostly based on the package by Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp> 
https://github.com/jsk-ros-pkg/jsk_3rdparty

## Tutorials

1. Install this package and vosk

  ```bash
  sudo apt install ros-${ROS_DISTRO}-ros-vosk
  ```
  
2. Launch speech recognition node

  ```bash
  roslaunch ros_vosk ros_vosk.launch
  ```
  
## Interface

### Publishing Topics

### Advertising Services

* `speech_recognition` (`speech_recognition_msgs/SpeechRecognition`)

  Service for speech recognition

## Parameters

* `~model` (`String`, default: `small-en-us-0.4`)

  Model to use for speech recognition
    
## Author

Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
Nickolay V. Shmyrev <nshmyrev@gmail.com>
