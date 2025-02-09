import numpy as np
from scipy.io import wavfile
from pydub import AudioSegment

# Audio parameters
duration = 3  # seconds
sample_rate = 44100  # Hz
frequency = 440  # Hz (A4 note)

# Generate time array
t = np.linspace(0, duration, int(sample_rate * duration))

# Create sine wave
audio_data = np.sin(2 * np.pi * frequency * t)

# Normalize to 16-bit range
audio_data = (audio_data * 32767).astype(np.int16)

# First save as WAV
wav_file = "temp_water.wav"
wavfile.write(wav_file, sample_rate, audio_data)

# Convert to MP3
audio = AudioSegment.from_wav(wav_file)
audio.export("water.mp3", format="mp3")

# Clean up temporary WAV file
import os
os.remove(wav_file)

print("Created water.mp3 with a 3-second sine wave tone")