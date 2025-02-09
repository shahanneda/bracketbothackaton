import os
import io
import wave
import threading
import pyaudio
from openai import OpenAI
from dotenv import load_dotenv
from ctypes import *
from contextlib import contextmanager
import paho.mqtt.client as mqtt
import time  # Make sure time is imported for the sleep calls

# Load environment variables from .env file
load_dotenv()

# Instantiate the OpenAI client using your API key.
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Add these constants after the load_dotenv() call
MQTT_BROKER_ADDRESS = "localhost"
MQTT_TOPIC = "robot/drive"

# Add MQTT client setup after the OpenAI client setup
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.connect(MQTT_BROKER_ADDRESS)
mqtt_client.loop_start()

# --- Optional: Suppress ALSA error messages ---
# These functions suppress a lot of ALSA (and related JACK) warnings.
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    # Simply pass to ignore ALSA errors.
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary("libasound.so")
    asound.snd_lib_error_set_handler(c_error_handler)
    try:
        yield
    finally:
        asound.snd_lib_error_set_handler(None)
# --- End optional suppression ---


def process_command(command):
    """
    Process the transcribed command (e.g., control your robot).
    Modify this function to add your control logic.
    """
    print("Command received:", command)
    if "never" in command.lower():
        print("Moving forward...")
        mqtt_client.publish(MQTT_TOPIC, "forward")
    elif "shreded" in command.lower():
        print("Moving backward...")
        mqtt_client.publish(MQTT_TOPIC, "backward")
        print("Apple!")
    elif "wheat" in command.lower():
        print("moving left")
        mqtt_client.publish(MQTT_TOPIC, "left")
        time.sleep(1.0)
        mqtt_client.publish(MQTT_TOPIC, "stop")
    elif "eat" in command.lower():
        print("moving right")
        mqtt_client.publish(MQTT_TOPIC, "right")
        time.sleep(1.0)
        mqtt_client.publish(MQTT_TOPIC, "stop")
    # Insert your robot control logic here.

def frames_to_wav_bytes(frames, channels, sample_rate, sample_width):
    """
    Convert raw audio frames into an in-memory WAV file.
    The API expects a file-like object with a valid filename extension.
    """
    buffer = io.BytesIO()
    with wave.open(buffer, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(sample_width)
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(frames))
    buffer.seek(0)
    buffer.name = "audio.wav"  # Filename helps the API infer the file type.
    return buffer

def transcribe_audio(frames, channels, sample_rate, sample_width):
    """
    Convert recorded audio frames to a WAV file and send them to the API.
    Uses the new client.audio.transcriptions.create() method.
    """
    audio_file = frames_to_wav_bytes(frames, channels, sample_rate, sample_width)
    try:
        result = client.audio.transcriptions.create(
            file=audio_file,
            model="whisper-1",
            response_format="text"
        )
        # Sometimes the API returns a string directly instead of an object with a .text attribute.
        if hasattr(result, "text"):
            transcription = result.text.strip()
        else:
            transcription = result.strip()
        if transcription:
            process_command(transcription)
    except Exception as e:
        print("Error during transcription:", e)

def record_audio():
    """
    Continuously record short audio segments from the microphone and,
    after each segment, spawn a thread to transcribe the audio without interrupting recording.
    """
    # Audio configuration.
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000  # A good sample rate for speech recognition.
    RECORD_SECONDS = 3  # Adjust duration to balance latency and accuracy.

    # Use our noalsaerr context manager to suppress many ALSA warnings.
    with noalsaerr():
        p = pyaudio.PyAudio()
    sample_width = p.get_sample_size(FORMAT)

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    print("Listening for commands... (Press Ctrl+C to stop)")
    try:
        while True:
            frames = []
            # Capture audio for the duration of RECORD_SECONDS.
            for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
                data = stream.read(CHUNK, exception_on_overflow=False)
                frames.append(data)
            # Process transcription in a separate thread to avoid gaps in recording.
            threading.Thread(target=transcribe_audio,
                             args=(frames, CHANNELS, RATE, sample_width)).start()
    except KeyboardInterrupt:
        print("Stopping recording...")
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()

if __name__ == "__main__":
    try:
        record_audio()
    finally:
        # Clean up MQTT client
        mqtt_client.publish(MQTT_TOPIC, "stop")
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print("MQTT client shutdown complete.")
