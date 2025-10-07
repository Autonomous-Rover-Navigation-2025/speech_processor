import rclpy
from rclpy.node import Node
import pyaudio
import speech_recognition as sr
import riva.client


class AudioProcessor(Node):
    def __init__(self):
        super().__init__('audio_processor')
        self.get_logger().info('🎙️ Audio Processor Node Started')

        # Audio config
        self.CHUNK = 1024
        self.RATE = 44100
        self.CHANNELS = 1
        self.FORMAT = pyaudio.paInt16
        self.RECORD_SECONDS = 5
        self.input_device_index = 0
        self.record_stream = None

        self.audio_interface = pyaudio.PyAudio()

        # Timer to record every 5 seconds
        self.timer = self.create_timer(5.0, self.record_and_transcribe)
        self.get_logger().info("🎙️ Recording will starts in 5 Secs...")

        # Speech recognizer instance
        self.recognizer = sr.Recognizer()
        # Authenticate and create Riva ASR client

        self.riva_auth = riva.client.Auth(uri="localhost:50051")  # adjust if remote
        self.riva_asr = riva.client.ASRService(self.riva_auth)

    def record_and_transcribe(self):
        self.get_logger().info("🎙️ Recording started... You Can speak Now, it will be take only first 5 Secs of talk  as an input for processing")

        record_stream = self.audio_interface.open(format=self.FORMAT,
                                           channels=self.CHANNELS,
                                           rate=self.RATE,
                                           input=True,
                                           input_device_index=self.input_device_index,
                                           frames_per_buffer=self.CHUNK)

        frames = []
        for _ in range(0, int(self.RATE / self.CHUNK * self.RECORD_SECONDS)):
            data = record_stream.read(self.CHUNK, exception_on_overflow=False)
            frames.append(data)

        record_stream.stop_stream()
        record_stream.close()
        self.get_logger().info("✅ Recording finished.")

        # Combine frames
        audio_bytes = b''.join(frames)
        # Wrap raw data in AudioData: (frame_data, sample_rate, sample_width)
        audio_data = sr.AudioData(audio_bytes, self.RATE, 2)  # 2 bytes per sample for paInt16
        self.get_logger().info("Processing your audio speech data")

        try:
            # text = self.recognizer.recognize_google(audio_data)
            # self.get_logger().info(f'🗣️ You said: "{text}"')

            response = self.riva_asr.offline_recognize(
                audio_bytes,
                sample_rate_hz=self.RATE,
                language_code="en-US"
            )

            text = response[0].transcript if response else ""
            if text:
                self.get_logger().info(f'🗣️ Riva STT: "{text}"')  

              # Convert to lowercase for case-insensitive match
            if "hey rover" in text.lower():
                self.get_logger().warn("🟢 Wake word 'hey rover' detected! Performing action...")

        # except sr.UnknownValueError:
        #     self.get_logger().info("🤔 Could not understand audio.")
        # except sr.RequestError as e:
        #     self.get_logger().error(f"🚫 Google API error: {e}")
        except Exception as e:
            self.get_logger().error(f"🚫 Riva STT error: {e}")

    def destroy_node(self):
        self.audio_interface.terminate()
        print("🔚 Audio resources (audio_interface) released.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AudioProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down the audio_processor ...")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()