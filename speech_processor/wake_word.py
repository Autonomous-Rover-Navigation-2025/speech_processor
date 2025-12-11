import os, time, numpy as np, sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from openwakeword.model import Model
import openwakeword
from scipy.signal import resample_poly 
from sounddevice import PortAudioError

MODEL_NAME = "hey_jarvis_v0.1.tflite"
TARGET_SR  = 16000         # model expects 16 kHz
FRAME_MS   = 80            # ~80 ms windows → 1280 samples @ 16k

class WakeWordNode(Node):
    def __init__(self):
        super().__init__("wake_word_node")

        # Params (defaults match your working environment)
        self.declare_parameter("device_index", 0)   # mic works at index 0
        self.declare_parameter("threshold", 0.35)   # start lower for bring-up
        self.declare_parameter("debounce_s", 1.5)
        self.declare_parameter("gain", 10.0)         # ~+18 dB; tune 5–12
        self.declare_parameter("log_scores", True)

        gp = self.get_parameter
        self.device_index = int(gp("device_index").value)
        self.threshold    = float(gp("threshold").value)
        self.debounce_s   = float(gp("debounce_s").value)
        self.gain         = float(gp("gain").value)
        self.log_scores   = bool(gp("log_scores").value)

        # Model
        openwakeword.utils.download_models()
        self.model = Model(wakeword_models=[MODEL_NAME])

        # Open the device at its native SR
        dinfo = sd.query_devices(self.device_index)
        self.dev_sr    = int(dinfo["default_samplerate"])
        self.dev_frame = int(self.dev_sr * (FRAME_MS / 1000.0))  # 80 ms @ dev SR

        self.stream = sd.InputStream(
            samplerate=self.dev_sr, channels=1, dtype="int16",
            blocksize=self.dev_frame, device=self.device_index
        )
        self.stream.start()

        # ROS pubs/subs/timers
        self.pub      = self.create_publisher(Empty, "/wake_word", 10)
        self.last_hit = 0.0
        self.timer    = self.create_timer(FRAME_MS / 1000.0, self.tick)

        self.get_logger().info(
            f"Listening 'hey jarvis' | dev={self.device_index} | dev_sr={self.dev_sr} "
            f"| thr={self.threshold} | gain={self.gain} | debounce={self.debounce_s}s"
        )

    def _to_16k_80ms(self, pcm_dev: np.ndarray) -> np.ndarray:
        """Amplify (if needed) and resample 80 ms of device PCM to 1280 samples @ 16 kHz."""
        x = pcm_dev.astype(np.float32)

        # Software gain for quiet USB mics (clip to int16)
        if self.gain != 1.0:
            x = np.clip(x * self.gain, -32768, 32767)

        if self.dev_sr == TARGET_SR:
            y = x
        elif self.dev_sr == 48000:
            # 48k → 16k: exact 1/3 downsample
            y = resample_poly(x, up=1, down=3)
        elif self.dev_sr == 44100:
            # 44.1k → 16k: exact rational resample
            y = resample_poly(x, up=160, down=441)
        else:
            # Generic rational resample to 16k
            y = resample_poly(x, up=TARGET_SR, down=self.dev_sr)

        # Ensure exact 1280 samples for 80 ms @ 16k
        target_len = int(TARGET_SR * (FRAME_MS / 1000.0))  # 1280
        if len(y) != target_len:
            y = y[:target_len] if len(y) > target_len else np.pad(y, (0, target_len - len(y)))

        return y.astype(np.int16)

    def tick(self):
        try:
            audio, _ = self.stream.read(self.dev_frame)
            pcm_dev = np.frombuffer(audio, dtype=np.int16)
            pcm16   = self._to_16k_80ms(pcm_dev)

            score = self.model.predict(pcm16).get(MODEL_NAME, 0.0)
            # if self.log_scores:
            #     self.get_logger().info(f"score={score:.3f}")

            now = time.time()
            if score >= self.threshold and (now - self.last_hit) > self.debounce_s:
                self.last_hit = now
                self.get_logger().info(f"wake HIT (score={score:.2f})")
                self.pub.publish(Empty())

        except PortAudioError as e:
            self.get_logger().error(f"Audio stream error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in tick(): {e}")

    def destroy_node(self):
        try:
            if hasattr(self, "stream") and self.stream:
                self.stream.stop()
                self.stream.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = WakeWordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
