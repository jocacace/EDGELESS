import pvporcupine
import pyaudio
import struct
import wave
import numpy as np
import requests
import base64

OUTPUT_FILENAME = "output.wav"  # Name of the output file
CHANNELS = 1  # Number of channels (1 for mono, 2 for stereo)
RATE = 48000  # Sample rate
RECORD_SECONDS = 5  # Duration of recording
CHUNK = 2048  # Number of audio frames per buffer
FORMAT = pyaudio.paInt16  # Format of the audio (16-bit int)

# Replace with your access key from Picovoice Console
ACCESS_KEY = "qBGaykD3Jkw5/RTWO/VbMGKDr9Evu+no4EAaNLn5bjhHdQRd/0bMxw=="

# Initialize the Porcupine wake word detector
ppn = pvporcupine.create(access_key=ACCESS_KEY, 
                               keywords=["alexa"], 
                               sensitivities=[0.1])

# Set up audio stream parameters
#pa = pyaudio.PyAudio()


p = pyaudio.PyAudio()


for i in range(p.get_device_count()):
    device_info = p.get_device_info_by_index(i)
    print(f"Device {i}: {device_info['name']}, Channels: {device_info['maxInputChannels']}")


stream = p.open(format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            input_device_index=5)

frames = []

done = False
is_speaking = False
silence_counter = 0
speaking_counter = 0
buffer = []
frames = []


counter_th = 5
voulme_th = 5
try:
    while not done:

        data = stream.read( CHUNK, exception_on_overflow=False) 

        audio_data = np.frombuffer(data, dtype=np.int16)

        # Calculate RMS (volume level)
        rms = np.sqrt(np.mean(audio_data**2))
        if( rms > voulme_th  ):
            if( speaking_counter > counter_th ):
                is_speaking = True
            else:
                speaking_counter = speaking_counter +1                
        elif is_speaking:
            if silence_counter > counter_th:
                done = True
            else:
                silence_counter = silence_counter + 1

    
        if( is_speaking ):
            print("Speaking")
            frames.append(data)        
        else: 
            print("No speaking")

    # Convert to float32, normalized to [-1.0, 1.0]
    raw_data = b''.join(frames)
    audio_data_int16 = np.frombuffer(raw_data, dtype=np.int16)
    
    audio_data_float32 = audio_data_int16 / np.iinfo(np.int16).max


    print("TO send: ", audio_data_float32, " ", len(audio_data_float32))
    print("Type: ", type(audio_data_float32) )
    
    url = "http://127.0.0.1:7035/"
    headers = {
        "Host": "demo.edgeless.com"
    }
    array_base64 = base64.b64encode(audio_data_float32.tobytes()).decode('utf-8')
    response = requests.post(url, headers=headers, json={"array": raw_data})
    #response = requests.post(url, headers=headers, data=data)

    print("La risposta 'e: ", response.text)
    
    # Convert back to int16 to save as WAV
    audio_data_int16 = (audio_data_float32 * np.iinfo(np.int16).max).astype(np.int16)
    with wave.open(OUTPUT_FILENAME, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(audio_data_int16.tobytes())

        print(f"Audio saved as {OUTPUT_FILENAME}")
    
    
except KeyboardInterrupt:
    print("Stopping wake word detection...")
finally:
    # Clean up resources
    stream.stop_stream()
    stream.close()
    p.terminate()
    porcupine.delete()

