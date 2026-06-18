# SynROV AiBot Python Package

This directory contains the Python runtime/interface for SynROV. It provides the Tkinter AiBot UI, a dedicated WebSocket AI runtime, safety filtering, replay/orchestration helpers, voice/audio/vision utilities, dataset collection, and learning helpers.

Install runtime dependencies from this directory:

```bash
python -m pip install -r requirements.txt
```

Run the Tkinter interface:

```bash
python synrov_aibot.py
```

Run the package entry point:

```bash
python -m synrov_aibot
```

Run the dedicated WebSocket runtime without the Tkinter UI:

```bash
python -m synrov_aibot --modern --uri ws://127.0.0.1:9000/
```

`PyAudio` is optional and can be installed separately when you want SpeechRecognition microphone capture through that backend.
