import pyaudio
import wave
import speech_recognition as sr
import re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# Parameters for audio recording
chunk = 1024
sample_format = pyaudio.paInt16
channels = 1
#fs = 44100
fs=8000
seconds = 3
filename = "output.wav"

number_words = {
    "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
    "seven": 7, "eight": 8, "nine": 9, "ten": 10
}

class SpeechMovementNode(Node):
    def __init__(self):
        super().__init__('speech_movement')
        self.goal_publisher = self.create_publisher(Pose, '/pose', 10)
    
    def record_audio(self):
        """Record audio and save it to a WAV file."""

        print("Recording...")
        p = pyaudio.PyAudio()

        stream = p.open(format=sample_format,
                        channels=channels,
                        rate=fs,
                        frames_per_buffer=chunk,
                        input=True)
        frames = [stream.read(chunk) for _ in range(0, int(fs / chunk * seconds))]
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
        """Transcribe speech from the recorded WAV file."""
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
        """Parse and execute coordinate commands."""
        command = self.convert_number_words_to_digits(command)  # Convert words to digits
        print(f"Processed command: '{command}'")  # Debugging statement

        # Updated regex to capture coordinates in the "X comma Y" format
        match = re.search(
            r"(?:move to|go to|go|move|i want to go|stop at)?\s*(\d+)\s*comma\s*(\d+)|(\d+)\s*comma\s*(\d+)",
            command
        )
        
        print(f"Regex match: {match}")  # Debugging statement

        if match:
            # Initialize x and y
            x, y = None, None

            # Check which groups are matched to extract coordinates
            if match.group(1) and match.group(2):  # Matches with command phrase
                x = float(match.group(1))
                y = float(match.group(2))
            elif match.group(3) and match.group(4):  # Matches without command phrase
                x = float(match.group(3))
                y = float(match.group(4))

            # Ensure x and y are valid
            if x is not None and y is not None:
                print(f"Moving to coordinates: ({x}, {y})")
                
                # Create Pose message with the coordinates
                goal_pose = Pose()
                goal_pose.position.x = float(x)
                goal_pose.position.y = float(y)
                
                # Publishing to the /goal_pose topic
                self.goal_publisher.publish(goal_pose)
                print("Goal published to /goal_pose")
            else:
                print(f"Could not extract coordinates from command: {command}")
        else:
            print(f"Unrecognized command: {command}")


def main(args=None):
    rclpy.init(args=args)
    speech_node = SpeechMovementNode()

    try:
        while True:
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
