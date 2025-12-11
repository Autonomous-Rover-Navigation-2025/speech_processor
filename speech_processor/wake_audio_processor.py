import os, time, threading, subprocess, wave
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty

import sounddevice as sd
from sounddevice import PortAudioError
from scipy.signal import resample_poly

import speech_recognition as sr
import openwakeword
from openwakeword.model import Model

# --- Wake-word model config ---
MODEL_NAME = "hey_jarvis_v0.1.tflite"
TARGET_SR  = 16000        # OWW expects 16 kHz
FRAME_MS   = 80           # 80 ms windows -> 1280 samples @ 16k
TARGET_LEN = TARGET_SR * FRAME_MS // 1000  # 1280

def _beep(freq=500, dur_ms=120, sr=44100):
    """Fire-and-forget short beep using aplay to avoid PortAudio output conflicts."""
    t = np.linspace(0, dur_ms/1000.0, int(sr*dur_ms/1000.0), False)
    pcm16 = (0.3*np.sin(2*np.pi*freq*t)*32767).astype(np.int16).tobytes()
    tmp = "/tmp/kws_beep.wav"
    with wave.open(tmp, "wb") as wf:
        wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(sr); wf.writeframes(pcm16)
    try:
        subprocess.Popen(["aplay", "-q", tmp])
    except Exception:
        pass

class WakeASRNode(Node):
    def __init__(self):
        super().__init__("wake_audio_processor")

        # ---- Parameters (mirrors your working settings) ----
        self.declare_parameter("device_index", 0)     # mic index
        self.declare_parameter("threshold", 0.35)     # wake sensitivity
        self.declare_parameter("debounce_s", 1.5)
        self.declare_parameter("gain", 1.0)          # software gain (clip safe)
        self.declare_parameter("log_scores", False)
        self.declare_parameter("asr_seconds", 5)      # record window after wake
        self.declare_parameter("asr_preroll_ms", 0)   # optional preroll, default 0

        gp = self.get_parameter
        self.device_index  = int(gp("device_index").value)
        self.threshold     = float(gp("threshold").value)
        self.debounce_s    = float(gp("debounce_s").value)
        self.gain          = float(gp("gain").value)
        self.log_scores    = bool(gp("log_scores").value)
        self.asr_seconds   = int(gp("asr_seconds").value)
        self.asr_preroll_ms = int(gp("asr_preroll_ms").value)

        # ---- Wake-word model ----
        openwakeword.utils.download_models()
        self.model = Model(wakeword_models=[MODEL_NAME])

        # ---- Recognizer (Google) ----
        self.recognizer = sr.Recognizer()

        # ---- Mic device info / native stream ----
        dinfo = sd.query_devices(self.device_index)
        self.dev_sr    = int(dinfo["default_samplerate"])
        
        #self.dev_sr = 44100
        self.dev_frame = int(self.dev_sr * (FRAME_MS / 1000.0))  # 80 ms at native SR

        # Single long-lived input stream
        self.stream = sd.InputStream(
            samplerate=self.dev_sr, channels=1, dtype="int16",
            blocksize=self.dev_frame, device=self.device_index
        )
        self.stream.start()

        # ---- ROS I/O ----
        self.pub_wake = self.create_publisher(Empty, "/wake_word", 10)
        self.pub_text = self.create_publisher(String, "/speech_text", 10)
        self.pub_state = self.create_publisher(String, "/wake_asr/state", 10)  # optional
        self.pub_tts = self.create_publisher(String, '/speak_text', 10)

        # State machine
        self.state = "WAKE"
        self.last_hit = 0.0

        # Timer loop (80 ms)
        self.timer = self.create_timer(FRAME_MS/1000.0, self._tick)

        self.get_logger().info(
            f"🎙️ Unified Wake+ASR started | dev={self.device_index} | dev_sr={self.dev_sr}  | dev_frame= {self.dev_frame}"
            f"| thr={self.threshold} | gain={self.gain} | debounce={self.debounce_s}s "
            f"| asr={self.asr_seconds}s"
        )

    # ---------- Main timer: wake-word listening ----------
    def _tick(self):
        if self.state != "WAKE":
            return

        try:
            audio, _ = self.stream.read(self.dev_frame)
            pcm_dev = np.frombuffer(audio, dtype=np.int16)
            pcm16   = self._to_16k_80ms(pcm_dev)

            score = self.model.predict(pcm16).get(MODEL_NAME, 0.0)
            if self.log_scores:
                self.get_logger().info(f"score={score:.3f}")

            now = time.time()
            if score >= self.threshold and (now - self.last_hit) > self.debounce_s:
                self.last_hit = now
                self.get_logger().info(f"🔥 Wake HIT (score={score:.2f})")
                self.pub_wake.publish(Empty())
                self.pub_tts.publish(String(data="Hello Anteater, how can I help you today?"))
                time.sleep(2)
                self._set_state("ASR")
                #_beep()
                # spin ASR capture/transcribe in a thread so timer keeps running
                threading.Thread(target=self._do_asr_once, daemon=True).start()

        except PortAudioError as e:
            self.get_logger().error(f"Audio stream error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in tick(): {e}")

    # ---------- ASR one-shot (record from same stream, then transcribe) ----------
    def _do_asr_once(self):
        try:
            # Optional short preroll if you want to catch trailing wake speech
            preroll = max(0, self.asr_preroll_ms) / 1000.0
            if preroll > 0:
                # read-and-discard preroll (still from same stream)
                n = int(self.dev_sr * preroll) // 1024
                for _ in range(n):
                    self.stream.read(1024)

            self.get_logger().info(f"🎙️ Recording command for {self.asr_seconds}s...")
            frames = []
            frames_needed = int(self.dev_sr / 1024 * self.asr_seconds)
            for _ in range(frames_needed):
                data, _ = self.stream.read(1024)
                frames.append(data)
            wav_bytes = b"".join(frames)

            # Save WAV at device SR (Google can handle 48k, 44.1k, etc.)
            wav_path = "/tmp/asr_input.wav"
            with wave.open(wav_path, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)          # int16
                wf.setframerate(self.dev_sr) # native SR
                wf.writeframes(wav_bytes)

            # Transcribe
            with sr.AudioFile(wav_path) as src:
                audio_data = self.recognizer.record(src)
            try:
                text = self.recognizer.recognize_google(audio_data, language="en-US")
                text_l = text.lower()
                self.get_logger().info(f'🗣️ Recognized: "{text_l}"')
                self._publish_text(text_l)
            except sr.UnknownValueError:
                self.get_logger().warn("🤔 Could not understand the audio.")
            except sr.RequestError as e:
                self.get_logger().error(f"🌐 Google API error: {e}")

        except Exception as e:
            self.get_logger().error(f"❌ ASR pipeline error: {e}")
        finally:
            # 🔒 1) Start a fresh debounce window
            self.last_hit = time.time()

            # 🧹 2) Hard-flush PortAudio by recreating the stream
            try:
                if getattr(self, "stream", None):
                    self.stream.stop()
                    self.stream.close()
            except Exception:
                pass
            try:
                self.stream = sd.InputStream(
                    samplerate=self.dev_sr, channels=1, dtype="int16",
                    blocksize=self.dev_frame, device=self.device_index
                )
                self.stream.start()
            except Exception as e:
                self.get_logger().error(f"❌ Failed to reopen mic stream: {e}")

            # 🎧 3) Back to wake listening
            self._set_state("WAKE")
            self.get_logger().info("🎧 Back to wake listening")
            
    # ---------- Helpers ----------
    def _publish_text(self, text: str):
        if not text:
            return
        msg = String(); msg.data = text
        self.pub_text.publish(msg)

    def _set_state(self, s: str):
        self.state = s
        self.pub_state.publish(String(data=s))

    def _to_16k_80ms(self, pcm_dev: np.ndarray) -> np.ndarray:
        """Apply gain and resample one 80 ms device frame to exactly 1280 @16k for OWW."""
        x = pcm_dev.astype(np.float32)
        if self.gain != 1.0:
            x = np.clip(x * self.gain, -32768, 32767)

        if self.dev_sr == TARGET_SR:
            y = x
        elif self.dev_sr == 48000:
            y = resample_poly(x, up=1, down=3)      # exact 48k -> 16k
        elif self.dev_sr == 44100:
            y = resample_poly(x, up=160, down=441)  # exact 44.1k -> 16k
        else:
            y = resample_poly(x, up=TARGET_SR, down=self.dev_sr)

        if len(y) != TARGET_LEN:
            if len(y) > TARGET_LEN:
                y = y[:TARGET_LEN]
            else:
                y = np.pad(y, (0, TARGET_LEN - len(y)))

        return y.astype(np.int16)

    # ---------- Cleanup ----------
    def destroy_node(self):
        try:
            if getattr(self, "stream", None):
                self.stream.stop()
                self.stream.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = WakeASRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
