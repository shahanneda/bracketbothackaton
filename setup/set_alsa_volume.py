import alsaaudio

def set_alsa_volume(volume=75):
    try:
        # Find the card number for UACDemoV1.0
        cards = alsaaudio.cards()
        card_num = None
        for i, card in enumerate(cards):
            if 'UACDemoV10' in card:
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



if __name__ == "__main__":
    set_alsa_volume()