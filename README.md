ROS2 Package: speech_processor

Voice interface for the Autonomous Rover Navigation project.
This package enables the rover to listen, understand intent, and respond verbally as part of a voice-driven autonomy system.

speech_processor is responsible for speech input and output in ROS 2. It bridges human voice interaction with downstream systems such as intent classification, navigation (Nav2), and dialogue via an LLM.

Overview

The voice system follows a layered pipeline:

Input → Processing → Decision → Output

Spoken commands are captured from a microphone, converted to text, routed based on intent (conversation vs navigation), and answered using synthesized speech.

Voice Interaction Flow

“Hey Jarvis” → wake_audio_processor → Speech-to-Text → Intent Classification
→ Small Talk → LLM (Ollama)
→ Navigation → Location Matcher / Nav2
→ /speech_reply → tts_publisher → Spoken Response

Input Layer
Wake Word Detection and Audio Capture

Node: wake_audio_processor
Libraries: PyAudio, OpenWakeWord

The system runs in a continuous listening loop, waiting for the wake word “Hey Jarvis.”
When the wake word is detected, the node captures a short audio segment from the microphone and forwards it for transcription.

The node automatically cycles between idle (wake) and active (listen) states, keeping the system responsive while minimizing unnecessary processing.

Processing Layer
Speech-to-Text (STT)

Captured audio is converted into text using an ASR engine (local Whisper or Google ASR depending on deployment).
The final transcription is published to the ROS topic:

/speech_text

This enables hands-free voice control of the rover.

Intent Classification

The transcribed text is analyzed to determine the user’s intent.
A lightweight transformer model (e.g. DistilBERT) classifies the input as either:

Conversational / Small Talk

Navigation / Command

This separation ensures that time-critical navigation logic is handled independently from conversational dialogue.

Decision and Communication Layer
Conversational Interaction (Small Talk)

Non-navigation queries are routed to a local LLM running via Ollama.
The LLM handles greetings, status questions, and general dialogue, then produces a text response for the rover to speak.

Navigation Commands

Navigation-related commands are forwarded to the autonomy stack, including:

Location matching using Sentence-BERT embeddings

Path planning and execution via Nav2

The system generates short confirmation messages such as “Heading to the garage” or “Destination reached,” which are passed back for speech output.

Output Layer
Text-to-Speech (TTS)

Node: tts_publisher
Engine: Piper TTS 

The tts_publisher node subscribes to the following topic:

/speech_reply

When a message arrives, it is sent to a Piper TTS engine running inside a Docker container.
The text is converted into natural-sounding speech, saved as an audio file, and immediately played through the Jetson’s speaker.

ROS Topics

/speech_text
Published by the speech-to-text pipeline. Contains the final transcription of user speech.

/speech_reply
Subscribed by the text-to-speech pipeline. Contains the text the rover should speak aloud.

The rover doesn’t just hear commands—it knows when to chat, when to drive, and when to stay quiet.
