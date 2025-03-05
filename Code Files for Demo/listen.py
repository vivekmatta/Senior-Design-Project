import pyaudio
import numpy as np
import serial
import time

# Set up Serial connection to ESP32 (Change COM port if needed)
ser = serial.Serial("/dev/tty.usbserial-58900065101", 115200, timeout=1)  # Change to your actual port

# Audio setup
CHUNK = 1024  # Number of audio samples per frame
FORMAT = pyaudio.paInt16  # 16-bit audio format
CHANNELS = 1  # Mono audio
RATE = 44100  # Sample rate (Hz)
THRESHOLD = 20000  # Clap detection threshold (adjustable)

p = pyaudio.PyAudio()
stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

print("Listening for claps...")

while True:
    data = np.frombuffer(stream.read(CHUNK, exception_on_overflow=False), dtype=np.int16)
    peak = np.abs(data).max()  # Get the loudest audio sample

    if peak > THRESHOLD:  # If the sound is loud enough, trigger the ESP32
        print("Clap detected! Sending signal to ESP32...")
        ser.write(b"start\n")  # Send "start" command to ESP32
        time.sleep(1)  # Prevent multiple triggers from the same clap

# Close audio stream
stream.stop_stream()
stream.close()
p.terminate()
ser.close()
