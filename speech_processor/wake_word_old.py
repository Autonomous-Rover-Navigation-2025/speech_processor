
import os
import queue
import json
import threading

import rclpy
from rclpy.node import Node
from vosk import Model, KaldiRecognizer
import sounddevice as sd

from speech_command.speech_control import SpeechMovementNode


class WakeWordListener(Node):
    def __init__(self):
        super().__init__('wake_word_listener')

        # # Load Vosk model from local directory inside package
        # script_dir = os.path.dirname(os.path.realpath(__file__))
        # model_path = os.path.join(script_dir, "../../vosk-model")
        # model_path = os.path.abspath(model_path)

        # Load Vosk model from absolute path
        model_path = "/workspaces/isaac_ros-dev/src/speech_command/vosk-model"

        self.get_logger().info(f"🔍 Loading Vosk model from: {model_path}")
        self.model = Model(model_path)

        self.recognizer = KaldiRecognizer(self.model, 16000)
        self.q = queue.Queue()
        self.keep_listening = True
        self.wake_word = "hey rover"

        self.audio_thread = threading.Thread(target=self.listen_audio, daemon=True)
        self.audio_thread.start()

    def listen_audio(self):
        def callback(indata, frames, time, status):
            if status:
                self.get_logger().warn(f"Audio status: {status}")
            self.q.put(bytes(indata))

        try:
            with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                                   channels=1, callback=callback):
                self.get_logger().info("🎧 Listening for 'hey rover'...")

                while self.keep_listening:
                    data = self.q.get()
                    if self.recognizer.AcceptWaveform(data):
                        result = json.loads(self.recognizer.Result())
                        text = result.get("text", "").lower()
                        self.get_logger().info(f"[Heard]: {text}")

                        if self.wake_word in text:
                            self.get_logger().info("🛎️ Wake word detected! Triggering command...")
                            self.trigger_speech_command()
        except Exception as e:
            self.get_logger().error(f"❌ Audio stream error: {e}")

    def trigger_speech_command(self):
        self.keep_listening = False

        try:
            command_node = SpeechMovementNode()
            command_node.record_audio()
            recognized_text = command_node.recognize_speech()
            if recognized_text:
                command_node.process_command(recognized_text)
        except Exception as e:
            self.get_logger().error(f"❌ Error in command processing: {e}")
        finally:
            self.keep_listening = True
            self.audio_thread = threading.Thread(target=self.listen_audio, daemon=True)
            self.audio_thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = WakeWordListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Shutting down wake word listener.")
    finally:
        node.keep_listening = False
        rclpy.shutdown()


if __name__ == '__main__':
    main()