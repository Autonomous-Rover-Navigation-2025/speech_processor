import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import speech_recognition as sr
import time
import wave
import os


class AudioProcessor(Node):
    def __init__(self):
        super().__init__('audio_processor')
        self.get_logger().info('🎙️ Audio Processor Node Started')

        # ===== Audio Config =====
        self.CHUNK = 1024
        self.RATE = 44100
        self.CHANNELS = 1
        self.FORMAT = pyaudio.paInt16
        self.RECORD_SECONDS = 5
        self.input_device_index = 0
        self.RECORD_INTERVAL = 5.0  # seconds between each recording cycle

        # ===== Global filename for saving audio =====
        self.output_filename = "audio_processor.wav"  # ✅ defined globally for use across methods

        # ===== Initialize Audio and Recognizer =====
        self.audio_interface = pyaudio.PyAudio()
        self.recognizer = sr.Recognizer()

        # ===== ROS 2 Publisher =====
        self.publisher_ = self.create_publisher(String, '/speech_text', 10)

        self.get_logger().info("✅ Initialization completed. Waiting to record...")

    # -----------------------------
    #  AUDIO RECORDING FUNCTION
    # -----------------------------
    def record_audio(self):
        """Record audio from the input device."""
        self.get_logger().info("🎙️ Recording started... Speak now (5 sec)...")

        stream = self.audio_interface.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            input_device_index=self.input_device_index,
            frames_per_buffer=self.CHUNK
        )

        frames = []
        for _ in range(0, int(self.RATE / self.CHUNK * self.RECORD_SECONDS)):
            data = stream.read(self.CHUNK, exception_on_overflow=False)
            frames.append(data)

        stream.stop_stream()
        stream.close()
        self.get_logger().info("✅ Recording finished.")
        return b"".join(frames)

    # -----------------------------
    #  SAVE TO WAV FILE FUNCTION
    # -----------------------------
    def save_audio_to_file(self, audio_bytes):
        """Save recorded audio bytes to a WAV file."""
        with wave.open(self.output_filename, 'wb') as wf:
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(self.audio_interface.get_sample_size(self.FORMAT))
            wf.setframerate(self.RATE)
            wf.writeframes(audio_bytes)
        self.get_logger().info(f"💾 Audio saved to {self.output_filename}")

    # -----------------------------
    #  GOOGLE SPEECH RECOGNITION FUNCTION
    # -----------------------------
    def recognize_speech(self):
        """Transcribe speech from the recorded WAV file."""
        if not os.path.exists(self.output_filename):
            self.get_logger().error(f"🚫 Audio file not found: {self.output_filename}")
            return None

        recognizer = sr.Recognizer()
        self.get_logger().info("🔄 Starting transcription using Google Speech Recognition...")

        with sr.AudioFile(self.output_filename) as source:
            audio_data = recognizer.record(source)

        try:
            text = recognizer.recognize_google(audio_data, language="en-US")
            self.get_logger().info(f'🗣️ You said: "{text}"')
            return text.lower()

        except sr.UnknownValueError:
            self.get_logger().warn("🤔 Could not understand the audio.")
            return None
        except sr.RequestError as e:
            self.get_logger().error(f"🚫 Google Speech Recognition API error: {e}")
            return None

    # -----------------------------
    #  PUBLISHING FUNCTION
    # -----------------------------
    def publish_transcription(self, text):
        """Publish the recognized text to the /speech_text topic."""
        if not text:
            self.get_logger().warn("⚠️ Empty text, not publishing.")
            return

        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f"📤 Published to /speech_text: '{text}'")

    # -----------------------------
    #  PROCESSING FUNCTION
    # -----------------------------
    def process_transcription(self, text):
        """Process recognized text (detect commands, wake word, etc.)."""
        self.get_logger().info("🧠 Entered process_transcription() function")

        if not text:
            self.get_logger().warn("⚠️ No transcription text received. Skipping processing.")
            return

        self.get_logger().info(f"📝 Received transcription: '{text}'")
        self.get_logger().debug(f"📏 Text length: {len(text)} characters")

        self.get_logger().info("📡 Publishing recognized text to /speech_text topic...")
        self.publish_transcription(text)
        self.get_logger().debug("✅ Text successfully published.")

        if "hey rover" in text.lower():
            self.get_logger().warn("🟢 Wake word 'hey rover' detected! Performing action...")

    # -----------------------------
    #  MAIN LOOP FUNCTION
    # -----------------------------
    def run(self):
        """Main recording-transcription loop."""
        try:
            while rclpy.ok():
                self.get_logger().info(f"⏳ Waiting {self.RECORD_INTERVAL} sec before next capture...")
                time.sleep(self.RECORD_INTERVAL)

                # 1️⃣ Record
                audio_data = self.record_audio()

                # 2️⃣ Save
                self.save_audio_to_file(audio_data)

                # 3️⃣ Recognize
                text = self.recognize_speech()

                # 4️⃣ Process + Publish
                self.process_transcription(text)

        except KeyboardInterrupt:
            self.get_logger().info("🛑 KeyboardInterrupt received. Stopping node.")
        finally:
            self.cleanup()

    # -----------------------------
    #  CLEANUP FUNCTION
    # -----------------------------
    def cleanup(self):
        """Release audio resources."""
        self.audio_interface.terminate()
        self.get_logger().info("🔚 Audio resources released.")
        self.destroy_node()


# ========================================================
#  MAIN ENTRY POINT
# ========================================================
def main(args=None):
    rclpy.init(args=args)
    node = AudioProcessor()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
