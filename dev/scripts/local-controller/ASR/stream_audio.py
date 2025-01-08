#!/usr/bin/env python3

import torch
import torchaudio
import pyaudio
from webrtc_noise_gain import AudioProcessor
import time
import numpy as np
import wave
import pvporcupine
from pvrecorder import PvRecorder
import sys
import os

audio = pyaudio.PyAudio()
torch.set_num_threads(1)

stream = audio.open(
        format=pyaudio.paInt16,
        channels=int(1),
        rate=16000,
        input=True,
        frames_per_buffer=1024,
        input_device_index=5
    )

FORMAT = pyaudio.paInt16
CHANNELS = 1
SAMPLE_RATE = 16000 #NOT CURRENTLY USED HERE, on register one
CHUNK = 1600           # 10 ms of audio (160 samples @ 16 KHz) #chunk = frame_length = 1600

def int2float(sound):
    abs_max = np.abs(sound).max()
    sound = sound.astype('float32')
    if abs_max > 0:
        sound *= 1/32768
    sound = sound.squeeze()  # depends on the use case
    return sound

def getpcm_vad( CHUNK, audio_processor, model, ppn):
        frames = []
        audio_chunk = stream.read(CHUNK, exception_on_overflow = False)
        divided_chunks = [audio_chunk[i * 320: (i + 1) * 320] for i in range(10)]

        for i in range(10):
            result = audio_processor.Process10ms(divided_chunks[i])
            audio_chunk = result.audio
            frames.append(np.frombuffer(audio_chunk, np.int16))
        pcm = np.concatenate(frames)
        audio_float32 = int2float(pcm)

        list_mean_conf = [model(torch.from_numpy(audio_float32[i:i + 512]), 16000).item() for i in range(0, len(audio_float32) - 64, 512)]
        mean_conf = sum(list_mean_conf)/len(list_mean_conf)   

        is_key_word_activated = any(ppn.process(pcm[i:i + 512]) >= 0 for i in range(0, len(pcm) - 64, 512))
        is_key_word_activated = is_key_word_activated or (ppn.process(pcm[-512:]) >= 0)
        
        return is_key_word_activated, pcm, mean_conf

def main():

    global device
    

	
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    CUSTOM_KEYWORDS = [script_dir + "/ppn/Hey-robot-one_en_linux_v3_0_0.ppn", script_dir + "/ppn/Hey-robot-two_en_linux_v3_0_0.ppn"]
    access_key='W6gkurkyc/3hzlVUV7pwSbGLvavnYE/KFnDL/+IKq66elKQNukQjqg=='
    ppn = pvporcupine.create(access_key=access_key, 
                            keyword_paths=CUSTOM_KEYWORDS, 
                            sensitivities=[0.5]*len(CUSTOM_KEYWORDS))

    model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', force_reload=False) 
    (get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks) = utils

    auto_gain_dbfs = 26
    # 0 = disable, 4 is max suppression
    noise_suppression_level = 4
    # 16 Khz mono with 16-bit samples only
    audio_processor = AudioProcessor(auto_gain_dbfs, noise_suppression_level)    
    is_recording = False
    
    audio_buffer = []
    
    while True:
        is_person_silenced = False
        is_person_silenced_forface = False

        is_keyword_activated, pcm, mean_conf = getpcm_vad( CHUNK, audio_processor, model, ppn)
        is_person_silenced = mean_conf < 0.65
        

        if is_keyword_activated and not is_recording:
            is_recording = True
            audio_buffer = []
            silence_start_time = None
            print("Start listening")
        elif is_keyword_activated and is_recording == True:
            audio_buffer = []

        if is_recording:
            audio_buffer.extend(pcm)                    
            if is_person_silenced:  # Adjust the threshold as needed
                if silence_start_time is None:
                    silence_start_time = time.time()
                elif time.time() - silence_start_time > 2.0:
                    is_recording = False
                    print("Recording is end")
                    
                    audio_buffer = np.array(audio_buffer, dtype=np.float32)/ 32768.0
                    print(audio_buffer)
                    
                    audio_buffer = []

            else:
                silence_start_time = None
            
        time.sleep(0.1)
                   
if __name__ == '__main__':
    global device       
    device = sys.argv[1] if len(sys.argv) > 1 else "cpu"
    print('using device', device)
    main()