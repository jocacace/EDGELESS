#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
#import base64
import whisper
import os
#import time
import numpy as np
#import wave
#import struct
from std_msgs.msg import Float32MultiArray

#import pickle   
#import pveagle
#import torch
#import rospkg
#from io import BytesIO
#import sys

class AutonmaticSpeechRecognition(Node):
    def __init__(self):
        super().__init__('automatic_speech_recognition')
        self.get_logger().info('Initializing ASR')        
        whisper_model = "small.en"
        save_model_path = "/home/jcacace/dev/EDGELESS/EDGELESS/dev/scripts/local-controller/ros2_ws/src/edgeless_asr/weights"
        self.whisper_model = whisper.load_model(whisper_model, download_root=save_model_path, device="cuda")
        
        print("Whisper model loaded")
        print(self.whisper_model)
        self.subscription = self.create_subscription(Float32MultiArray, 'audio_output_topic', self.audio_callback, 10)

    def audio_callback(self, msg):
        audio_buffer = np.array(msg.data, dtype=np.float32)        
        result = self.whisper_model.transcribe(audio_buffer)
        print( result["text"], "en" )
        
    
def main(args=None):
    rclpy.init(args=args)
    node = AutonmaticSpeechRecognition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    global device   
    device = "cuda"
    main()