from pi5neo import Pi5Neo
import time

def iranian_flag(neo):
    # Calculate section sizes (dividing strip into 3 parts)
    section_size = neo.num_leds // 3
    
    # Green section (top of flag)
    for i in range(0, section_size):
        neo.set_led_color(i, 0, 255, 0)  # Green
    
    # White section (middle of flag)
    for i in range(section_size, section_size * 2):
        neo.set_led_color(i, 255, 255, 255)  # White
    
    # Red section (bottom of flag)
    for i in range(section_size * 2, neo.num_leds):
        neo.set_led_color(i, 255, 0, 0)  # Red
    
    neo.update_strip()

neo = Pi5Neo('/dev/spidev0.0', 10, 800)
iranian_flag(neo)