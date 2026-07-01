#!/usr/bin/env python3
"""Test the robot's TTS (and optionally the OpenRouter LLM) on a dev PC — no ROS, no
robot, no rosbridge. Drives the same ``web_control.tts`` / ``web_control.llm`` modules
the robot uses; both are deliberately ROS-free so they import and run standalone.

TTS picks a backend automatically: the robot's ``pico2wave``+``aplay`` if installed,
else the OS speech engine — **Windows SAPI** (PowerShell ``System.Speech``) or macOS
``say`` — so you actually hear the line on your laptop.

Examples (run with any Python 3; no extra packages needed):

    python scripts/dev_tts_test.py "Hello, I am Nano."
    python scripts/dev_tts_test.py --voice de-DE --speed 120 "Guten Tag!"
    # Full pipeline (set your key first, or drop it in memory/openrouter_key):
    export OPENROUTER_API_KEY=sk-or-...
    python scripts/dev_tts_test.py --llm "tell me a short robot joke"
    python scripts/dev_tts_test.py --llm --model anthropic/claude-haiku-4.5 \
        --persona "You are cheeky and love bad puns." "say hello"

On Windows PowerShell, set the key with:  $env:OPENROUTER_API_KEY = "sk-or-..."
(or put it, one line, in memory/openrouter_key -- gitignored.)
"""
import argparse
import os
import sys

# Import the robot's modules straight from src/ (they have no rclpy dependency).
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src", "web_control"))

from web_control.tts import TtsEngine            # noqa: E402
from web_control.llm import LlmClient, MOODS     # noqa: E402


def _load_openrouter_key():
    """$OPENROUTER_API_KEY wins; else load it from a one-line memory/openrouter_key file
    (gitignored) so this can be run without exporting the env var every session."""
    if os.environ.get("OPENROUTER_API_KEY", "").strip():
        return
    _root = os.path.join(_HERE, "..")
    for path in (os.path.join(_root, "memory", "openrouter_key"),
                 os.path.join(_HERE, ".openrouter_key")):
        if os.path.isfile(path):
            with open(path, encoding="utf-8") as f:
                key = f.read().strip()
            if key:
                os.environ["OPENROUTER_API_KEY"] = key
            return


def main():
    ap = argparse.ArgumentParser(description="Speak a line (and optionally generate it "
                                             "with OpenRouter) on a dev PC.")
    ap.add_argument("text", nargs="*", help="line to speak, or the prompt when --llm")
    ap.add_argument("--llm", action="store_true",
                    help="generate the line + mood via OpenRouter (needs OPENROUTER_API_KEY)")
    ap.add_argument("--model", default="",
                    help="OpenRouter model id (default nvidia/nemotron-3-ultra-550b-a55b:free)")
    ap.add_argument("--persona", default="", help="extra in-character system prompt")
    ap.add_argument("--voice", default="en-US", help="en-US | en-GB | de-DE")
    ap.add_argument("--volume", type=int, default=100)
    ap.add_argument("--speed", type=int, default=100)
    ap.add_argument("--pitch", type=int, default=100)
    args = ap.parse_args()
    text = " ".join(args.text).strip()

    tts = TtsEngine(default_voice=args.voice, on_word=_print_word, logger=_log)
    if not tts.available():
        print("! No TTS backend found. Install pico2wave (Linux) — on Windows/macOS the "
              "built-in SAPI/`say` engine should be auto-detected.", file=sys.stderr)
        return 2
    tts.configure(voice=args.voice, volume=args.volume, speed=args.speed, pitch=args.pitch)

    mood = None
    if args.llm:
        _load_openrouter_key()
        client = LlmClient(enabled=True, api_key="", model=args.model, persona=args.persona,
                           logger=_log)               # api_key="" -> OPENROUTER_API_KEY env
        if not client.available():
            print("! LLM unavailable: set OPENROUTER_API_KEY in your environment.",
                  file=sys.stderr)
            return 2
        prompt = text or "Say one short, friendly, spontaneous line and pick a fitting mood."
        print(f"… asking {client.model} …")
        reply = client.generate(prompt)
        if not reply:
            print("! No reply from the model (check the key/model/network).", file=sys.stderr)
            return 1
        text, mood = reply["say"], reply["mood"]
        print(f"\n  say : {text}\n  mood: {mood}   (one of {', '.join(MOODS)})\n")
    elif not text:
        print("! Nothing to say. Pass some text, or use --llm to generate it.",
              file=sys.stderr)
        return 2

    print(f"speaking [{tts.voice}]: ", end="", flush=True)
    tts.say(text)
    t = getattr(tts, "_thread", None)                 # block until the utterance finishes
    if t is not None:
        t.join()
    print("\ndone.")
    return 0


def _print_word(word):
    # The TTS karaoke callback: prints each word as it's "spoken" (blank = end).
    if word:
        print(word, end=" ", flush=True)


def _log(msg):
    print(f"[tts] {msg}", file=sys.stderr)


if __name__ == "__main__":
    sys.exit(main())
