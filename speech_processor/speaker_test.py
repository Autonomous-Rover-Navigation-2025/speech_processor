import pyaudio
import wave
import time
import sys
import os
import audioop  # for resampling and conversion

# -----------------------------
# Configuration
# -----------------------------
WAVE_OUTPUT_FILENAME = "orbison_singer.wav"  # your existing .wav file
FORMAT = pyaudio.paInt16
CHUNK = 1024


def main():
    audio = pyaudio.PyAudio()
    play_stream = None
    wf = None

    try:
        # --- List devices ---
        print("\n--- Available Audio Devices ---")
        for i in range(audio.get_device_count()):
            info = audio.get_device_info_by_index(i)
            print(f"{i}: {info['name']} - Input Channels: {info['maxInputChannels']}, Output Channels: {info['maxOutputChannels']}")
        print("--------------------------------")

        # --- Automatically select first output-capable device ---
        output_device_index = None
        for i in range(audio.get_device_count()):
            info = audio.get_device_info_by_index(i)
            if info['maxOutputChannels'] > 0:
                output_device_index = i
                print(f"✅ Auto-selected output device index: {output_device_index} ({info['name']})")
                break

        if output_device_index is None:
            print("❌ No output-capable audio device found.")
            sys.exit(1)

        # --- Check for existing WAV file ---
        if not os.path.exists(WAVE_OUTPUT_FILENAME):
            print(f"❌ File not found: {WAVE_OUTPUT_FILENAME}")
            sys.exit(1)

        # --- Load output device info ---
        output_device_info = audio.get_device_info_by_index(output_device_index)
        print(f"\n🔊 Selected Output Device: {output_device_index} - {output_device_info['name']}")
        print("📋 Device Properties:")
        print(f"  🔊 Channels (Max Output): {output_device_info['maxOutputChannels']}")
        print(f"  🎚️ Default Sample Rate: {output_device_info['defaultSampleRate']}")
        device_rate = int(output_device_info['defaultSampleRate'])

        # --- Open WAV file ---
        print(f"\n📂 Loading file: {WAVE_OUTPUT_FILENAME}")
        wf = wave.open(WAVE_OUTPUT_FILENAME, 'rb')
        orig_rate = wf.getframerate()
        orig_width = wf.getsampwidth()
        orig_channels = wf.getnchannels()

        print(f"📄 WAV File Properties:")
        print(f"   - Channels       : {orig_channels}")
        print(f"   - Sample Width   : {orig_width} bytes")
        print(f"   - Frame Rate     : {orig_rate} Hz")
        print(f"   - Frame Count    : {wf.getnframes()}")
        print(f"   - Duration       : {wf.getnframes() / orig_rate:.2f} seconds")

        # --- Read all frames ---
        data = wf.readframes(wf.getnframes())

        # --- Resample if needed ---
        if orig_rate != device_rate:
            print(f"⚙️ Resampling from {orig_rate} Hz → {device_rate} Hz ...")
            data, _ = audioop.ratecv(data, orig_width, orig_channels, orig_rate, device_rate, None)
        else:
            print("✅ No resampling needed.")

        # --- Convert to 16-bit if 8-bit ---
        if orig_width == 1:
            print("🎚️ Converting 8-bit → 16-bit audio ...")
            data = audioop.bias(data, 1, 128)  # center 8-bit samples
            data = audioop.lin2lin(data, 1, 2)
            orig_width = 2

        # --- Ensure at least 1 output channel ---
        out_channels = min(orig_channels, output_device_info['maxOutputChannels'])
        if out_channels < 1:
            out_channels = 1

        # --- Playback ---
        print("\n🔊 Starting playback...")
        play_stream = audio.open(format=audio.get_format_from_width(orig_width),
                                 channels=out_channels,
                                 rate=device_rate,
                                 output=True,
                                 output_device_index=output_device_index)

        # play in chunks
        pos = 0
        while pos < len(data):
            play_stream.write(data[pos:pos + CHUNK])
            pos += CHUNK

        print("✅ Playback finished successfully.")

        # --- Cleanup ---
        play_stream.stop_stream()
        play_stream.close()
        wf.close()
        play_stream = None
        wf = None

    except Exception as e:
        print(f"❌ Error occurred: {e}")

    finally:
        if play_stream:
            play_stream.stop_stream()
            play_stream.close()
        if wf:
            wf.close()
        audio.terminate()
        print("🔚 Audio resources released.")


if __name__ == "__main__":
    main()
