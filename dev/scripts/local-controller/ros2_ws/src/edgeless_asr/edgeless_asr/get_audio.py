#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import torch
import torchaudio
import pyaudio
from webrtc_noise_gain import AudioProcessor
import time
import numpy as np
import pvporcupine
import os
import sys

from std_msgs.msg import Float32MultiArray

def int2float(sound):
    abs_max = np.abs(sound).max()
    sound = sound.astype('float32')
    if abs_max > 0:
        sound *= 1 / 32768
    sound = sound.squeeze()
    return sound


class AudioProcessorNode(Node):
    def __init__(self):
        super().__init__('audio_processor_node')
        self.get_logger().info('Initializing Audio Processor Node')

        # Parameters
        self.device = self.declare_parameter("device", "cpu").value
        self.access_key = self.declare_parameter("access_key", "W6gkurkyc/3hzlVUV7pwSbGLvavnYE/KFnDL/+IKq66elKQNukQjqg==").value
        self.chunk = self.declare_parameter("chunk_size", 1600).value

        # Initialize audio processor components
        script_dir = os.path.dirname(os.path.abspath(__file__))
        script_dir = "/home/jcacace/dev/EDGELESS/EDGELESS/dev/scripts/local-controller/ASR"
        custom_keywords = [
            f"{script_dir}/ppn/Hey-robot-one_en_linux_v3_0_0.ppn",
            f"{script_dir}/ppn/Hey-robot-two_en_linux_v3_0_0.ppn"
        ]

        self.ppn = pvporcupine.create(
            access_key=self.access_key,
            keyword_paths=custom_keywords,
            sensitivities=[0.5] * len(custom_keywords)
        )

        self.model, self.utils = torch.hub.load(
            repo_or_dir='snakers4/silero-vad', model='silero_vad', force_reload=False
        )
        auto_gain_dbfs = 26
        # 0 = disable, 4 is max suppression
        noise_suppression_level = 4
        self.audio_processor = AudioProcessor(auto_gain_dbfs, noise_suppression_level)

        self.is_recording = False
        self.audio_buffer = []
        self.silence_start_time = None

        # PyAudio setup
        self.audio = pyaudio.PyAudio()
        torch.set_num_threads(1)
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024,
            input_device_index=4
        )
        self.audio_pub = self.create_publisher(Float32MultiArray, 'audio_output_topic', 10)
        
        # ROS Timer
        self.timer = self.create_timer(0.1, self.process_audio)

    def process_audio(self):
        is_keyword_activated, pcm, mean_conf = self.get_pcm_vad()

        is_person_silenced = mean_conf < 0.65
        

        if is_keyword_activated and not self.is_recording:
            self.is_recording = True
            self.audio_buffer = []
            self.silence_start_time = None
            self.get_logger().info("Start listening")

        elif is_keyword_activated and self.is_recording:
            self.audio_buffer = []

        if self.is_recording:
            self.audio_buffer.extend(pcm)

            if is_person_silenced:
                if self.silence_start_time is None:
                    self.silence_start_time = time.time()
                elif time.time() - self.silence_start_time > 2.0:
                    self.is_recording = False
                    audio_buffer_normalized = np.array(self.audio_buffer, dtype=np.float32) / 32768.0                    

                    msg = Float32MultiArray()
                    msg.data = audio_buffer_normalized
                  
                    self.audio_buffer = []                                    
                    self.audio_pub.publish( msg )

            else:
                self.silence_start_time = None

    def get_pcm_vad(self):
        frames = []
        audio_chunk = self.stream.read(self.chunk, exception_on_overflow=False)
        divided_chunks = [audio_chunk[i * 320: (i + 1) * 320] for i in range(10)]

        for chunk in divided_chunks:
            result = self.audio_processor.Process10ms(chunk)
            audio_chunk = result.audio
            frames.append(np.frombuffer(audio_chunk, np.int16))
        pcm = np.concatenate(frames)
        audio_float32 = int2float(pcm)

        list_mean_conf = [
            self.model(torch.from_numpy(audio_float32[i:i + 512]), 16000).item()
            for i in range(0, len(audio_float32) - 64, 512)
        ]
        mean_conf = sum(list_mean_conf) / len(list_mean_conf)

        is_keyword_activated = any(
            self.ppn.process(pcm[i:i + 512]) >= 0 for i in range(0, len(pcm) - 64, 512)
        ) or self.ppn.process(pcm[-512:]) >= 0

        return is_keyword_activated, pcm, mean_conf

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
