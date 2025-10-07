import pyaudio
import wave
import speech_recognition as sr
import re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Set parameters for recording
chunk = 1024  # Record in chunks of 1024 samples
# chunk = 4096      #vaibhav
sample_format = pyaudio.paInt16  # 16 bits per sample
channels = 1
fs = 44100  # Record at 44100 samples per second, works on pc
# fs = 16000      #Vaibhav
#fs = 8000	# Works on raspberry pi
seconds = 3
filename = "output.wav"

# Mapping of number words to digits
number_words = {
    "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
    "seven": 7, "eight": 8, "nine": 9, "ten": 10
}

class SpeechMovementNode(Node):
    def __init__(self):
        super().__init__('speech_movement')
        #for turtle sim 
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        #for rasp pi
        self.publisher1_ = self.create_publisher(Twist, '/command', 10)
    
    def record_audio(self):
        """Record audio and save it to a WAV file."""
        print("Recording...")
        p = pyaudio.PyAudio()  # Create an interface to PortAudio

        # for i in range(p.get_device_count()):
        #     info = p.get_device_info_by_index(i)
        #     print(f"{i}: {info['name']}")
            
        stream = p.open(format=sample_format, 
                        channels=channels,
                        rate=fs,
                        frames_per_buffer=chunk,
                        input=True)
        frames = []
        for _ in range(0, int(fs / chunk * seconds)):
            data = stream.read(chunk)
            # data = stream.read(chunk, exception_on_overflow=False)      #vaibhav
            frames.append(data)

        stream.stop_stream()
        stream.close()
        print("Recording complete!")

        wf = wave.open(filename, 'wb')
        wf.setnchannels(channels)
        wf.setsampwidth(p.get_sample_size(sample_format))
        wf.setframerate(fs)
        wf.writeframes(b''.join(frames))
        wf.close()

    def recognize_speech(self):
        """Transcribe the speech from the recorded WAV file."""
        recognizer = sr.Recognizer()
        with sr.AudioFile(filename) as source:
            print("Transcribing speech...")
            audio_data = recognizer.record(source)
        try:
            text = recognizer.recognize_google(audio_data, language="en-US")
            print(f"You said: {text}")
            return text.lower()
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand the audio")
            return None
        except sr.RequestError as e:
            print(f"Could not request results from Google Speech Recognition service; {e}")
            return None

    def convert_number_words_to_digits(self, command):
        """Convert number words to digits in the command string."""
        for word, digit in number_words.items():
            command = command.replace(word, str(digit))
        return command

    def process_command(self, command):
        """Process the recognized voice command and move the TurtleBot accordingly."""
        command = self.convert_number_words_to_digits(command)
        pattern = re.compile(r"(forward|backward|right|left)\s*(\d+)?\s*(steps|times)?|(\d+)\s*(steps|times)?\s*(forward|backward|right|left)")
        match = pattern.search(command)

        twist = Twist()

        if match:
            if match.group(1):
                action = match.group(1)
                steps = int(match.group(2)) if match.group(2) else 1
            elif match.group(4):
                steps = int(match.group(4))
                action = match.group(6)

            for _ in range(steps):
                if action == "forward":
                    twist.linear.x = 10.0
                    twist.angular.z = 0.0
                elif action == "backward":
                    twist.linear.x = -10.0
                    twist.angular.z = 0.0
                elif action == "left":
                    twist.linear.x = 0.0
                    twist.angular.z = 5.0
                elif action == "right":
                    twist.linear.x = 0.0
                    twist.angular.z = -5.0

                self.publisher_.publish(twist)
                self.publisher1_.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.1)  # Process messages non-blocking

            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)  # Ensure stop is processed
        else:
            if command in ["forward", "backward", "right", "left"]:
                print(f"Moving {command}")
                if command == "forward":
                    twist.linear.x = 10.0
                    twist.angular.z = 0.0
                elif command == "backward":
                    twist.linear.x = -10.0
                    twist.angular.z = 0.0
                elif command == "left":
                    twist.linear.x = 0.0
                    twist.angular.z = 5.0
                elif command == "right":
                    twist.linear.x = 0.0
                    twist.angular.z = -5.0

                self.publisher_.publish(twist)
                self.publisher1_.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.1)
            else:
                print(f"Unrecognized command: {command}")

def main(args=None):
    rclpy.init(args=args)
    speech_node = SpeechMovementNode()

    try:
        while True:  # Run continuously until manually terminated
            speech_node.record_audio()
            speech_text = speech_node.recognize_speech()

            if speech_text:
                speech_node.process_command(speech_text)

    except KeyboardInterrupt:
        print("Shutting down the speech command node.")

    speech_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
