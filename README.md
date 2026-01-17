# ROS 2 Package: `speech_processor`

**Voice interface for the Autonomous Rover Navigation**

`speech_processor` provides the speech input and output layer for a voice-driven autonomous rover. It allows the rover to listen for spoken commands, understand intent, and respond verbally, while cleanly interfacing with downstream systems such as navigation (Nav2) and conversational dialogue via a local LLM.

---

## Overview

The voice system follows a layered pipeline: Input → Processing → Decision → Output

Spoken commands are captured from a microphone, converted to text, routed based on intent (conversation vs navigation), and answered using synthesized speech.

---

## Architecture

### Input Layer  
**Wake Word Detection & Audio Capture**

**Node:** `wake_audio_processor`  
**Libraries:** PyAudio, OpenWakeWord  

- Runs in a continuous listening loop.
- Waits for the wake word **“Hey Jarvis.”**
- Upon detection, captures a short audio segment from the microphone.
- Automatically cycles between:
  - **Idle (wake)** state
  - **Active (listen)** state  

This design keeps the system responsive while minimizing unnecessary processing.

---

### Processing Layer

#### Speech-to-Text (STT)

- Captured audio is converted into text using an ASR engine:
  - Google ASR
- The final transcription is published to: /speech_text

This enables hands-free voice control of the rover.

---

#### Intent Classification

- Transcribed text is analyzed to determine user intent.
- A lightweight transformer model (e.g. **DistilBERT**) classifies input as:
  - **Conversational / Small Talk**
  - **Navigation / Command**

This separation ensures time-critical navigation logic is handled independently from conversational dialogue.

---

## Decision & Communication Layer

### Conversational Interaction (Small Talk)

- Non-navigation queries are routed to a local LLM via **Ollama**.
- The LLM handles:
  - Greetings
  - Status questions
  - General dialogue
- Produces a text response for the rover to speak.

---

### Navigation Commands

- Navigation-related commands are forwarded to the autonomy stack:
  - Location matching using **Sentence-BERT embeddings**
  - Path planning and execution via **Nav2**
- Generates short confirmation messages such as:
  - “Heading to the Building.”
  - “Destination reached.”
- These messages are passed back for speech output.

---

## Output Layer

### Text-to-Speech (TTS)

**Node:** `tts_publisher`  
**Engine:** Piper TTS 

- Subscribes to: /speech_reply


- When a message arrives:
  - Text is sent to a Piper TTS engine running in a Docker container.
  - Speech is synthesized into an audio file.
  - Audio is immediately played through the Jetson’s speaker.

Running Piper in a container isolates heavy dependencies while keeping speech generation fast and reliable.

---

## ROS Topics

### `/speech_text`
- **Published by:** Speech-to-Text pipeline  
- **Description:** Final transcription of user speech  

### `/speech_reply`
- **Subscribed by:** Text-to-Speech pipeline  
- **Description:** Text the rover should speak aloud  

---

## Summary

`speech_processor` provides a clean, modular voice interface for ROS 2.
The rover doesn’t just hear commands — it knows when to chat, when to drive, and when to stay quiet.





