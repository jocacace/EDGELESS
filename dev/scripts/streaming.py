import pyaudio
import wave
import base64
import requests
import json
from io import BytesIO
import pvporcupine 
import numpy as np

# Configurazione di PyAudio
FORMAT = pyaudio.paInt16
CHANNELS = 1
#RATE = 16000
RATE = 48000
CHUNK = 1600
RECORD_SECONDS = 6
import openwakeword
from openwakeword.model import Model

# One-time download of all pre-trained models (or only select models)
openwakeword.utils.download_models()

# Instantiate the model(s)
model = Model(
    #wakeword_models=["path/to/model.tflite"],  # can also leave this argument empty to load all of the included pre-trained models
)

# Replace with your access key from Picovoice Console
ACCESS_KEY = "qBGaykD3Jkw5/RTWO/VbMGKDr9Evu+no4EAaNLn5bjhHdQRd/0bMxw=="

# Initialize the Porcupine wake word detector
ppn = pvporcupine.create(access_key=ACCESS_KEY, 
                               keywords=["alexa"], 
                               sensitivities=[0.1])

def record_audio():
    p = pyaudio.PyAudio()
    
    # Apri lo stream

    stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=RATE, #Jabra
                input=True,
                input_device_index=5)
    print("Recording...")
    
    frames = []
    
    # Registra l'audio
    for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        
        print("Len data: ", len(data))
        pcm = np.frombuffer(data, np.int16)

        print(pcm, len(pcm))
        is_key_word_activated = any(ppn.process(pcm[i:i + 512]) >= 0 for i in range(0, len(pcm) - 64, 512))
        print( is_key_word_activated )
        frames.append( np.frombuffer(data, np.int16) )
    
    '''
    print("Frames: ", frames)
    pcm = np.concatenate(frames)
    print("pcm: ", pcm)
    print("SIze pcm: ", len(pcm))
    is_key_word_activated = any(ppn.process(pcm[i:i + 512]) >= 0 for i in range(0, len(pcm) - 64, 512))
    print( is_key_word_activated )
    print("Recording finished.")
    '''
    # Chiudi lo stream
    stream.stop_stream()
    stream.close()
    p.terminate()
    
    # Scrivi l'audio in memoria come file WAV
    buffer = BytesIO()
    with wave.open(buffer, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
    buffer.seek(0)
    with wave.open("StramingFile.wav", 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
    return buffer

def send_audio_as_json(audio_buffer):
    # Codifica il file audio in base64
    audio_base64 = base64.b64encode(audio_buffer.read()).decode('utf-8')
    
    # Costruisci il JSON payload
    url = "http://127.0.0.1:7035/"
    headers = {
        "Host": "demo.edgeless.com"
    }
    
    # Invia il JSON al server
    #data=json.dumps(audio_base64)
    response = requests.post(url, json={"audio": audio_base64} , headers=headers)
    print(f"Response: {response.status_code}, {response.text}")

if __name__ == "__main__":
    audio_buffer = record_audio()
    print("Audio buffer: ", audio_buffer)
    send_audio_as_json(audio_buffer)
 