import os
import wave
import audioop
import pyttsx3

def generate_embedded_wav(text: str, output_filename: str):
    """
    Generates a WAV file optimized for microcontrollers (16kHz, 16-bit, Mono PCM)
    using the native Windows TTS engine.
    """
    # Ensure filename ends with .wav
    if not output_filename.lower().endswith('.wav'):
        output_filename += '.wav'
        
    raw_wav = "temp_raw_system.wav"
    
    # 1. Initialize the native Windows speech engine
    engine = pyttsx3.init()
    
    # Try to select a female voice (Zira is standard on Windows)
    voices = engine.getProperty('voices')
    for voice in voices:
        if "female" in voice.name.lower() or "zira" in voice.name.lower():
            engine.setProperty('voice', voice.id)
            break
            
    # Set speech rate (150 is natural and clear)
    engine.setProperty('rate', 150) 
    
    print(f"Generating voice for: \"{text}\"")
    engine.save_to_file(text, raw_wav)
    engine.runAndWait()
    
    # 2. Process and downsample the audio to exact embedded specs
    if not os.path.exists(raw_wav):
        print("Error: Temporary audio file was not generated.")
        return

    with wave.open(raw_wav, 'rb') as src:
        src_params = src.getparams()
        src_bytes = src.readframes(src_params.nframes)
        
        # Track initial properties
        current_width = src_params.sampwidth
        
        # Convert stereo to mono if the system voice defaults to stereo
        if src_params.nchannels == 2:
            mono_bytes = audioop.tomono(src_bytes, current_width, 1, 1)
        else:
            mono_bytes = src_bytes
            
        # Resample from native rate down to 16000 Hz
        resampled_bytes, _ = audioop.ratecv(
            mono_bytes, 
            current_width, 
            1, 
            src_params.framerate, 
            16000, 
            None
        )
        
        # Ensure it is explicitly converted to 16-bit PCM (2 bytes per sample)
        if current_width == 1:
            # 8-bit to 16-bit
            final_bytes = audioop.lin2lin(resampled_bytes, 1, 2)
        elif current_width == 3:
            # 24-bit to 16-bit
            final_bytes = audioop.lin2lin(resampled_bytes, 3, 2)
        elif current_width == 4:
            # 32-bit to 16-bit
            final_bytes = audioop.lin2lin(resampled_bytes, 4, 2)
        else:
            final_bytes = resampled_bytes

    # 3. Write final clean 16kHz, 16-bit, Mono PCM WAV file
    with wave.open(output_filename, 'wb') as dst:
        dst.setnchannels(1)      # Mono
        dst.setsampwidth(2)      # 16-bit PCM (2 bytes)
        dst.setframerate(16000)  # 16 kHz
        dst.writeframes(final_bytes)
        
    # Clean up temporary file
    if os.path.exists(raw_wav):
        os.remove(raw_wav)
        
    print(f"Success! '{output_filename}' created (16kHz, 16-bit, Mono WAV).\n")

if __name__ == "__main__":
    # Get interactive inputs from the terminal
    user_text = input("Enter the text to speak: ")
    user_filename = input("Enter the output filename (e.g., low_battery): ")
    
    generate_embedded_wav(user_text, user_filename)