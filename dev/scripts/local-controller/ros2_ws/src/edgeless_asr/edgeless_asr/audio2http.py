#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import base64
import whisper
import os
import time
import numpy as np
import wave
import struct
from std_msgs.msg import Float32MultiArray
import numpy as np
import pickle   
import pveagle
import torch
import rospkg
from io import BytesIO
import sys
import requests

class ToHTTP(Node):
    def __init__(self):
        super().__init__('toHttp')
        self.subscription = self.create_subscription(Float32MultiArray, 'audio_output_topic', self.audio_callback, 10)

    def audio_callback(self, msg):              
        
        audio_buffer = np.array(msg.data, dtype=np.float32)        
                
        url = "http://127.0.0.1:7035/"
        headers = {
            "Host": "demo.edgeless.com",
            "Content-Type": "application/json"
        }
        data = {
            "audio":  audio_buffer.tolist() 
        }

        response = requests.post(url, headers=headers, json=data)

        print(f"Status Code: {response.status_code}")
        print(f"Response Text: {response.text}")
    
def main(args=None):
    rclpy.init(args=args)
    node = ToHTTP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':        
    main()