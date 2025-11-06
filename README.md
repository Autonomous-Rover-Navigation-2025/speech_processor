# 🗣️ ROS2 Package: `speech_processor`

Voice interface for the **Autonomous Rover Navigation** project.  
It lets the rover **listen** using speech-to-text and **speak** using text-to-speech.  
Built on ROS2, Whisper ASR, and Piper TTS.

---

### 🎧 Speech Pipeline

"Hey Jarvis" → [wake_audio_processor] → /speech_text → /speech_reply → [tts_publisher] → Spoken Response            

---

The **speech-to-text (STT)** part of this package runs a continuous listening loop through the `wake_audio_processor` node. It waits for the wake word “Hey Jarvis.” When the wake word is detected, the node captures a few seconds of audio from the microphone and processes it using a local Whisper ASR model. The result is a text transcript of what the user said, which is then published to the `/speech_text` topic. This approach allows hands-free voice control for the rover. The node cycles automatically between “wake” and “listen” states, keeping the system responsive while saving processing power when idle.

The **text-to-speech (TTS)** side runs through the `tts_publisher` node. It subscribes to `/speech_reply`, which carries any text message the rover needs to speak aloud. When a message arrives, the node sends it to a running Piper TTS engine inside a Docker container, where the text is converted into natural-sounding speech and saved as an audio file. That file is immediately played through the Jetson’s speaker. This design keeps voice generation smooth and fast, offloading heavy synthesis to the container environment. Together, these two nodes create a seamless voice interface.
