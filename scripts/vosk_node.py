#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Intergrated by Angelo Antikatzidis https://github.com/a-prototype/vosk_ros
# Source code based on https://github.com/alphacep/vosk-api/blob/master/python/example/test_microphone.py from VOSK's example code

# Tuned for the python flavor of VOSK: vosk-0.3.31
# If you do not have vosk then please install it by running $ pip3 install vosk
# If you have a previous version of vosk installed then update it by running $ pip3 install vosk --upgrade
# Tested on ROS Noetic & Melodic. Please advise the "readme" for using it with ROS Melodic 

# This is a node that intergrates VOSK with ROS and supports a TTS engine to be used along with it
# When the TTS engine is speaking some words, the recognizer will stop listenning to the audio stream so it won't listen to it self :)

# It publishes to the topic speech_recognition/vosk_result a custom "speech_recognition" message
# It publishes to the topic speech_recognition/final_result a simple string
# It publishes to the topic speech_recognition/partial_result a simple string


import os
import sys
import json
import queue
import vosk
import sounddevice as sd
from mmap import MAP_SHARED

import rospy
import rospkg
from ros_vosk.msg import speech_recognition
from std_msgs.msg import String, Bool

import vosk_ros_model_downloader as downloader

class vosk_sr():
    def __init__(self):
        rospack = rospkg.RosPack()
        rospack.list()
        package_path = rospack.get_path('ros_vosk')
        
        model_path = '/models/'
        model_dir = package_path + model_path
        model = "None" #change the name of the model to match the downloaded model's name
        
        if not os.path.exists(model_dir+model):
            print ("No model found! Please use the GUI to download a model...")
            model_downloader = downloader.model_downloader()
            model_downloader.execute()
            model = model_downloader.model_to_download
        
        self.tts_status = False

        # ROS node initialization
        
        self.pub_vosk = rospy.Publisher('speech_recognition/vosk_result',speech_recognition, queue_size=10)
        self.pub_final = rospy.Publisher('speech_recognition/final_result',String, queue_size=10)
        self.pub_partial = rospy.Publisher('speech_recognition/partial_result',String, queue_size=10)

        self.rate = rospy.Rate(100)

        rospy.on_shutdown(self.cleanup)

        model_name = rospy.get_param('vosk/model',model)
        if not rospy.has_param('vosk/model'):
            rospy.set_param('vosk/model', model_name)

        self.msg = speech_recognition()

        self.q = queue.Queue()

        self.input_dev_num = sd.query_hostapis()[0]['default_input_device']
        if self.input_dev_num == -1:
            rospy.logfatal('No input device found')
            raise ValueError('No input device found, device number == -1')

        device_info = sd.query_devices(self.input_dev_num, 'input')
        # soundfile expects an int, sounddevice provides a float:
        
        self.samplerate = int(device_info['default_samplerate'])
        rospy.set_param('vosk/sample_rate', self.samplerate)

        self.model = vosk.Model(model_dir+model_name)

        #TODO GPUInit automatically selects a CUDA device and allows multithreading.
        # gpu = vosk.GpuInit() #TODO

    
    def cleanup(self):
        rospy.logwarn("Shutting down VOSK speech recognition node...")
    
    def stream_callback(self, indata, frames, time, status):
        #"""This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        self.q.put(bytes(indata))
        
    def tts_get_status(self,msg):
        self.tts_status = msg.data

    def tts_status_listenner(self):
        rospy.Subscriber('/tts/status', Bool, self.tts_get_status)

    def speech_recognize(self):    
        try:

            with sd.RawInputStream(samplerate=self.samplerate, blocksize=16000, device=self.input_dev_num, dtype='int16',
                               channels=1, callback=self.stream_callback):
                rospy.logdebug('Started recording')
                
                rec = vosk.KaldiRecognizer(self.model, self.samplerate)
                print("Vosk is ready to listen!")
                isRecognized = False
                isRecognized_partially = False
                
            
                while not rospy.is_shutdown():
                    self.tts_status_listenner()

                    if self.tts_status == True:
                        # If the text to speech is operating, clear the queue
                        with self.q.mutex:
                            self.q.queue.clear()
                        rec.Reset()

                    elif self.tts_status == False:
                        data = self.q.get()
                        if rec.AcceptWaveform(data):

                            # In case of final result
                            result = rec.FinalResult()

                            diction = json.loads(result)
                            lentext = len(diction["text"])

                            if lentext > 2:
                                result_text = diction["text"]
                                rospy.loginfo(result_text)
                                isRecognized = True
                            else:
                                isRecognized = False
                            # Resets current results so the recognition can continue from scratch
                            rec.Reset()
                        else:
                            # In case of partial result
                            result_partial = rec.PartialResult()
                            if (len(result_partial) > 20):

                                isRecognized_partially = True
                                partial_dict = json.loads(result_partial)
                                partial = partial_dict["partial"]

                        if (isRecognized is True):

                            self.msg.isSpeech_recognized = True
                            self.msg.time_recognized = rospy.Time.now()
                            self.msg.final_result = result_text
                            self.msg.partial_result = "unk"
                            self.pub_vosk.publish(self.msg)
                            rospy.sleep(0.1)
                            self.pub_final.publish(result_text)
                            isRecognized = False


                        elif (isRecognized_partially is True):
                            if partial != "unk":
                                self.msg.isSpeech_recognized = False
                                self.msg.time_recognized = rospy.Time.now()
                                self.msg.final_result = "unk"
                                self.msg.partial_result = partial
                                self.pub_vosk.publish(self.msg)
                                rospy.sleep(0.1)
                                self.pub_partial.publish(partial)
                                partial = "unk"
                                isRecognized_partially = False
                                


        except Exception as e:
            exit(type(e).__name__ + ': ' + str(e))
        except KeyboardInterrupt:
            rospy.loginfo("Stopping the VOSK speech recognition node...")
            rospy.sleep(1)
            print("node terminated")

if __name__ == '__main__':
    try:
        rospy.init_node('vosk', anonymous=False)
        rec = vosk_sr()
        rec.speech_recognize()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        rospy.logfatal("Error occurred! Stopping the vosk speech recognition node...")
        rospy.sleep(1)
        print("node terminated")
