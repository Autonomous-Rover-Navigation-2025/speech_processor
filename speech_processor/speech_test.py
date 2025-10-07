import pyaudio
import wave
import time
import sys

FORMAT = pyaudio.paInt16
RATE = 44100
CHUNK = 1024
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "test_output.wav"


def main():
    audio = pyaudio.PyAudio()
    input_device_index = 0
    output_device_index = 30
    record_stream = None
    play_stream = None
    wf = None

    try:
        # --- List devices and select defaults ---
        print("\n--- Available Audio Devices ---")
        for i in range(audio.get_device_count()):
            info = audio.get_device_info_by_index(i)
            print(f"{i}: {info['name']} - Input Channels: {info['maxInputChannels']}, Output Channels: {info['maxOutputChannels']}")
        print("--------------------------------")

        # --- check the Input and Output device index ---
        if input_device_index is None or output_device_index is None:
            print("❌ No suitable input/output device found.")
            sys.exit(1)

        # --- give the selected Input device configuartion details ---
        input_device_info = audio.get_device_info_by_index(input_device_index)
        print(f"\n🎙️ Selected Input Device: {input_device_index} - {input_device_info['name']}")
        print("📋 Device Properties:")
        print(f"  🔢 Channels (Max Input): {input_device_info['maxInputChannels']}")
        print(f"  🔊 Channels (Max Output): {input_device_info['maxOutputChannels']}")
        print(f"  🎚️ Default Sample Rate: {input_device_info['defaultSampleRate']}")
        print(f"  📟 Host API: {audio.get_host_api_info_by_index(input_device_info['hostApi'])['name']}")
        print(f"  🆔 Device Index: {input_device_info['index']}")

        # --- give the selected Output device configuartion details ---
        output_device_info = audio.get_device_info_by_index(output_device_index)
        print(f"\n🎙️ Selected Input Device: {output_device_index} - {output_device_info['name']}")
        print("📋 Device Properties:")
        print(f"  🔢 Channels (Max Input): {output_device_info['maxInputChannels']}")
        print(f"  🔊 Channels (Max Output): {output_device_info['maxOutputChannels']}")
        print(f"  🎚️ Default Sample Rate: {output_device_info['defaultSampleRate']}")
        print(f"  📟 Host API: {audio.get_host_api_info_by_index(output_device_info['hostApi'])['name']}")


        # --- Record ---
        print("\n🎙️ Recording...")
        record_stream = audio.open(format=FORMAT, channels=1, rate=RATE, input=True, input_device_index=input_device_index,
                            frames_per_buffer=CHUNK)

        frames = []
        for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = record_stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)

        print("✅ Recording finished")
        record_stream.stop_stream()
        record_stream.close()
        record_stream = None  # Mark as closed
        print("🔚 record stream resources released.")

        # --- Save ---
        wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(audio.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        wf = None  # Mark as closed
        print("🔚 .Wav File resources released.")
        print(f"💾 Saved to '{WAVE_OUTPUT_FILENAME}'")

        time.sleep(1)

        # --- Playback ---
        print("\n🔊 Playing back...")
        wf = wave.open(WAVE_OUTPUT_FILENAME, 'rb')
        print(f"📄 WAV File for  Playing back Properties:")
        print(f"   - Channels       : {wf.getnchannels()}")
        print(f"   - Sample Width   : {wf.getsampwidth()} bytes")
        print(f"   - Frame Rate     : {wf.getframerate()} Hz")
        print(f"   - Frame Count    : {wf.getnframes()}")
        print(f"   - Duration       : {wf.getnframes() / wf.getframerate():.2f} seconds")

        play_stream = audio.open(format=audio.get_format_from_width(wf.getsampwidth()),
                                channels=wf.getnchannels(),
                                rate=wf.getframerate(),
                                output=True,
                                output_device_index=output_device_index)

        data = wf.readframes(CHUNK)
        while data:
            play_stream.write(data)
            data = wf.readframes(CHUNK)

        print("✅ Playback finished")
        play_stream.stop_stream()
        play_stream.close()
        play_stream = None  # Mark as closed
        print("🔚 play stream resources released.")
        wf.close()
        print("🔚 .Wav File resources released.")
        wf = None

    except Exception as e:
        print(f"❌ Error occurred: {e}")

    finally:
        # Clean up safely
        if record_stream:
            record_stream.stop_stream()
            record_stream.close()
            print("🔚 record stream resources released.")
        if play_stream:
            play_stream.stop_stream()
            play_stream.close()
            print("🔚 play stream resources released.")
        if wf:
            wf.close()
            print("🔚 .Wav File resources released.")
        if audio:
            audio.terminate()
            print("🔚 Audio resources released.")

if __name__ == "__main__":
    main()