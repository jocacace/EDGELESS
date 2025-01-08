#!/usr/bin/env python3

#import rospkg
#import logging
#import io
#import shutil
#import numpy as np
#import torch
#torch.set_num_threads(1)
#import torchaudio
#torchaudio.set_audio_backend("soundfile")
import pyaudio
#from hydra import compose, initialize
#from webrtc_noise_gain import AudioProcessor
#from multiprocessing import shared_memory
#import rospy
#from std_msgs.msg import String, Bool, ColorRGBA
#import os
#import time
#import numpy as np
#import wave
#import struct
#from threading import Thread
#import pvporcupine
#from pvrecorder import PvRecorder
#from eut_hr_dialog_manager_asr_msgs.msg import Float32MultiArray
#from eut_hr_dialog_manager_asr.register_voice_action import RegisterVoice
#import hydra
#from hydra import compose, initialize
#import sys


audio = pyaudio.PyAudio()



for i in range(audio.get_device_count()):
    device_info = audio.get_device_info_by_index(i)
    print(f"Device {i}: {device_info['name']}, Channels: {device_info['maxInputChannels']}")

'''{'index': 5, 'structVersion': 2, 'name': 'Jabra SPEAK 510 USB: Audio (hw:2,0)', 'hostApi': 0, 'maxInputChannels': 1, 'maxOutputChannels': 2, 'defaultLowInputLatency': 0.024, 'defaultLowOutputLatency': 0.024, 'defaultHighInputLatency': 0.096, 'defaultHighOutputLatency': 0.096, 'defaultSampleRate': 16000.0}
''' 