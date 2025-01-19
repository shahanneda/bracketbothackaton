import os
import time
from io import BytesIO

import alsaaudio
import numpy as np
import soundfile as sf
import sounddevice as sd
from scipy import signal
from dotenv import load_dotenv

from openai import OpenAI
from elevenlabs.client import ElevenLabs
from transformers import pipeline
from transformers.pipelines.audio_utils import ffmpeg_microphone_live


load_dotenv()
PERPLEXITY_API_KEY = os.getenv('PERPLEXITY_API_KEY')
ELEVEN_LABS_API_KEY = os.getenv('ELEVEN_LABS_API_KEY')


device = 'cpu'

# TODO: Can you find/make a faster wakeword model?
classifier = pipeline(
    # "audio-classification", model="MIT/ast-finetuned-speech-commands-v2", device=device
    "audio-classification", model="0xb1/wav2vec2-base-finetuned-speech_commands-v0.02", device=device # faster
)

model = "openai/whisper-tiny.en"
# model = "distil-whisper/distil-small.en"
transcriber = pipeline(
    "automatic-speech-recognition", model=model, device=device
)


def set_alsa_volume(volume=75):
    try:
        cards = alsaaudio.cards()
        card_num = None
        for i, card in enumerate(cards):
            if 'Device' in card:
            # if 'UACDemoV10' in card:
                card_num = i
                break
        
        if card_num is None:
            print("Could not find UACDemoV1.0 audio device")
            return
            
        mixer = alsaaudio.Mixer('PCM', cardindex=card_num)
        mixer.setvolume(volume)
        print(f"Set UACDemoV1.0 volume to {volume}%")
    except alsaaudio.ALSAAudioError as e:
        print(f"Error setting volume: {e}")


def speak_text(text, voice="Brian"):
    """Generates audio from the given text using ElevenLabs API and plays through audio device."""

    # Check for special phrases that use pre-recorded audio
    text_normalized = text.lower().strip('-.,!?')
    if text_normalized in ["wow", "uhhhhhhhhh lemme think about that"]:
        # Load and play the corresponding audio file
        filename = "wow.wav" if text_normalized == "wow" else "uhh.wav"
        try:
            data, sample_rate = sf.read(filename, dtype='float32')
            device_name = "pulse"
            # device_name = "UACDemoV1.0"
            device_info = sd.query_devices(device_name, 'output')
            device_id = device_info['index']
            device_sample_rate = device_info['default_samplerate']

            # Resample if needed
            if sample_rate != device_sample_rate:
                number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
                data = signal.resample(data, number_of_samples)
                sample_rate = device_sample_rate

            sd.play(data, samplerate=sample_rate, device=device_id)
            sd.wait()
            return
        except Exception as e:
            pass
    
    # Initialize ElevenLabs client
    client = ElevenLabs(api_key=ELEVEN_LABS_API_KEY)
    
    # Generate audio from text
    audio = client.generate(
        text=text,
        voice=voice,
        model="eleven_multilingual_v2"
    )
    
    # Set audio device
    device_name = "pulse"  # Name of the USB audio device
    # device_name = "UACDemoV1.0"  # Name of the USB audio device
    device_info = sd.query_devices(device_name, 'output')
    device_id = device_info['index']
    device_sample_rate = device_info['default_samplerate']

    # Prepare audio data
    audio_data = b''.join(audio)
    data, sample_rate = sf.read(BytesIO(audio_data), dtype='float32')

    # Resample if needed
    if sample_rate != device_sample_rate:
        number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
        data = signal.resample(data, number_of_samples)
        sample_rate = device_sample_rate

    try:
        # Play the audio
        sd.play(data, samplerate=sample_rate, device=device_id)
        sd.wait()
        
        # Save to wav if text was "wow" or "uhh" (cache for future use)
        if text_normalized in ["wow", "uhhhhhhhhh lemme think about that"]:
            filename = "wow.wav" if text_normalized == "wow" else "uhh.wav"
            sf.write(filename, data, int(sample_rate))
            
    except sd.PortAudioError as e:
        print(f"Error playing audio: {e}")


def speak_text_stream(text, voice="Brian"):
    #TODO: Implement a fully streamed version of speak_text which plays audio as it is streamed from ElevenLabs API
    pass


def wait_for_wow(
    prob_threshold=0.5,
    debug=True,
):
    sampling_rate = classifier.feature_extractor.sampling_rate

    mic = ffmpeg_microphone_live(
        sampling_rate=sampling_rate,
        chunk_length_s=0.75,
        stream_chunk_s=0.5,
    )

    wake_word = 'wow'
    if wake_word not in classifier.model.config.label2id.keys():
        raise ValueError(
            f"Wake word {wake_word} not in set of valid class labels, pick a wake word in the set {classifier.model.config.label2id.keys()}."
        )


    print("Listening for wake word...")
    for prediction in classifier(mic):
        prediction = prediction[0]
        if debug:
            print(prediction)
        if prediction["label"] == wake_word:
            if prediction["score"] > prob_threshold:
                mic.close()
                return True


def transcribe(chunk_length_s=5.0, stream_chunk_s=2.0):
    sampling_rate = transcriber.feature_extractor.sampling_rate
    # sampling_rate = classifier.feature_extractor.sampling_rate

    print("Start speaking...")
    try:
        mic = None
        while True:
            if mic:
                mic.close()
            wait_for_wow()
            speak_text('WOW!!')
            print('WOW')
            weather_said = False
            start_time = time.time()
            mic = ffmpeg_microphone_live(
                sampling_rate=sampling_rate,
                chunk_length_s=chunk_length_s,
                stream_chunk_s=stream_chunk_s,
            )
            while not weather_said and (time.time()-start_time < 15):
                for item in transcriber(mic, generate_kwargs={"max_new_tokens": 128}):
                    # sys.stdout.write("\033[K")
                    current_text = item["text"].lower()
                    print(current_text)

                    # Check if "weather" appears or disappears between transcriptions
                    if "weather" in current_text:
                        speak_text("fine... I'll get the forecast I guess...")
                        on_weather_change()
                        weather_said = True
                        break
                        # print(current_text)
                    
                    if current_text.endswith('?'):
                        speak_text("uhhhhhhhhh lemme think about that...")
                        answer = ask_perplexity(current_text)
                        print(answer)
                        speak_text(answer)
                        weather_said = True
                        break
                    
                    
                    if not item["partial"][0]:
                        print()  # Move to next line when chunk is complete
                        break

            if not weather_said:
                speak_text("Hey what did you say WOW for!")
            
    except KeyboardInterrupt:
        print("\nStopped by user")


def format_weather_info(data):
    """Format weather data into a human-readable string."""
    current = data['current']
    
    # Extract relevant information
    temp = current['temp']
    feels_like = current['feels_like']
    weather = current['weather'][0]['description']
    humidity = current['humidity']
    wind_speed = current['wind_speed']
    
    # Construct weather message
    message = f"\nCurrent weather conditions:"
    message += f"\nTemperature: {temp}°C (feels like {feels_like}°C)"
    message += f"\nConditions: {weather}"
    message += f"\nHumidity: {humidity}%"
    message += f"\nWind Speed: {wind_speed} m/s\n"
    
    return message


def on_weather_change():
    try:
        weather = weather_perplexity()
        print(weather)
        speak_text(weather)
        
    except Exception as e:
        print(f"\nError getting weather: {str(e)}\n")


def weather_perplexity():
    client = OpenAI(api_key=PERPLEXITY_API_KEY, base_url="https://api.perplexity.ai")
    
    messages = [
        {
            "role": "system",
            "content": (
                "You are a weather assistant. Give a simple weather report using only "
                "common words that can be easily spoken and heard. Use exactly one "
                "sentence. Do not include dates and use the word degrees instead of "
                "the degrees symbol."
            ),
        },
        {
            "role": "user", 
            "content": (
                "Tell me the current temperature and weather conditions in Waterloo, "
                "Ontario in one simple sentence."
            ),
        },
    ]

    try:
        response = client.chat.completions.create(
            model="llama-3.1-sonar-large-128k-online",
            messages=messages,
        )
        
        # Extract the weather information from the response
        weather_info = response.choices[0].message.content
        return weather_info
    
    except Exception as e:
        return f"Error getting weather information: {str(e)}"
    

def ask_perplexity(question):
    client = OpenAI(api_key=PERPLEXITY_API_KEY, base_url="https://api.perplexity.ai")
    messages = [
        {
            "role": "system", 
            "content": (
                "You're a chill chatbot who keeps things super casual and brief. Just give me one quick sentence "
                "using everyday language, like you're texting a friend. I'm going to just ask stupid questions, try to give me chill answers"
            ),
        },
        {
            "role": "user",
            "content": question,
        },
    ]

    try:
        response = client.chat.completions.create(
            model="llama-3.1-sonar-large-128k-online", 
            messages=messages,
        )
        
        return response.choices[0].message.content
    
    except Exception as e:
        return f"Error getting response: {str(e)}"


set_alsa_volume(100)
transcribe()