#Boot sequence for robot head
#The "Health Status Check" script
#I am on linux, I have ALSA installed I also have python-alsaaudio installed
#I have a speaker connected to my computer
#I have a microphone connected to my computer

import alsaaudio
import wave
import numpy as np
import time
from pydub import AudioSegment
from pydub.playback import play

def check_speaker():
    print("Checking speaker...")
    try:
        # Load a placeholder MP3 file
        sound = AudioSegment.from_mp3("boot-media/boot.mp3")
        
        # Play the sound
        play(sound)
        
        print("Speaker test completed. Did you hear the sound?")
        return True
    except Exception as e:
        print(f"Speaker error: {str(e)}")
        return False

def check_microphone():
    print("Checking microphone...")
    try:
        # Set up the microphone for recording
        inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NONBLOCK, device='default')
        
        # Try to get the current hardware parameters
        try:
            channels = inp.channels()
            rate = inp.rate()
            format = inp.format()
            period_size = inp.periodsize()
        except AttributeError:
            # If auto-detection fails, set parameters manually for the ReSpeaker v2
            channels = 6
            rate = 16000
            format = alsaaudio.PCM_FORMAT_S16_LE
            period_size = 1024
            
            inp.setchannels(channels)
            inp.setrate(rate)
            inp.setformat(format)
            inp.setperiodsize(period_size)
        
        print(f"Microphone settings: {channels} channels, {rate} Hz, format {format}, period size {period_size}")

        # Load the test sound
        sound = AudioSegment.from_mp3("boot-media/boot.mp3")
        
        # Play the sound
        play(sound)
        
        # Record for the duration of the sound plus a small buffer
        duration = len(sound) / 1000.0  # Convert milliseconds to seconds
        start_time = time.time()
        frames = []

        while time.time() - start_time < duration + 0.5:  # Add 0.5 second buffer
            l, data = inp.read()
            if l:
                frames.append(data)
                time.sleep(0.001)

        # Save the recorded audio to a WAV file for verification
        with wave.open('recorded_audio.wav', 'wb') as wf:
            wf.setnchannels(channels)
            wf.setsampwidth(alsaaudio.PCM_FORMAT_TO_WIDTH(format))
            wf.setframerate(rate)
            wf.writeframes(b''.join(frames))

        # Check if audio was detected
        if len(frames) > 0:
            print("Microphone test completed. Audio detected and saved as 'recorded_audio.wav'")
            return True
        else:
            print("Microphone error: No audio detected")
            return False

    except Exception as e:
        print(f"Microphone error: {str(e)}")
        return False

# Main boot sequence (partial)
def main():
    checks = [
        ("Speaker", check_speaker),
        ("Microphone", check_microphone),
        # ... other checks ...
    ]
    
    total_checks = len(checks)
    completed_checks = 0
    errors = []

    for name, check_func in checks:
        result = check_func()
        completed_checks += 1
        
        if not result:
            errors.append(name)
            print(f"Error in {name} check")
            time.sleep(2)

    if errors:
        print(f"Boot Failed. Errors in: {', '.join(errors)}")
    else:
        print("Boot Success: All Systems OK")

if __name__ == "__main__":
    main()
