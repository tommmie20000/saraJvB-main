import os
import sys
import subprocess
import requests
import speech_recognition as sr
from faster_whisper import WhisperModel
import unicodedata
import threading
import time
import re
import cv2
import base64

# --- CONFIGURATIE ---
OLLAMA_URL = "http://192.168.1.40:11434/api/chat"
TEXT_MODEL = "llama3.1:8b"         # Krachtig Nederlands model
VISION_MODEL = "llama3.2-vision:11b" # Vision model

WAKE_WORDS = ["he sara", "hey sara", "hallo sara", "sara", "sarah"] 
EXIT_TRIGGER = "!?doei"
USER_EXIT_WORDS = ["doei", "dag", "tot ziens", "hou op", "stop"]

# --- PIPER ---
PIPER_PATH = "./piper/piper" 
VOICE_MODEL = "nl_NL-pim-medium.onnx" 

# --- PERSOONLIJKHEID ---
SYSTEM_PROMPT = f"""
Je bent Sara, de robot van het Jan van Brabant College. 
Houd je antwoorden kort (max 15 woorden). 
Als men vraagt 'wat zie je?' of 'kijk eens', antwoord dan ALTIJD met de code: ?!snapshot, gebruik dan niet direct de context die eerder aan je gegeven is.
"""

def get_snapshot():
    """Maakt een foto en geeft base64 terug."""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        return None
    # Korte pauze voor camera initialisatie
    time.sleep(0.5)
    success, frame = cap.read()
    if success:
        # Resize naar 640x480 voor snellere verwerking door de GPU
        frame = cv2.resize(frame, (640, 480))
        _, buffer = cv2.imencode('.jpg', frame)
        img_str = base64.b64encode(buffer).decode('utf-8')
        cap.release()
        return img_str
    cap.release()
    return None

def chat_with_ollama(user_input, history, image_b64=None):
    selected_model = VISION_MODEL if image_b64 else TEXT_MODEL
    
    # Als we een foto sturen, gebruiken we een SCHONE prompt zonder geschiedenis
    # om te voorkomen dat ze in een "?!snapshot" lus terechtkomt.
    if image_b64:
        messages = [{"role": "user", "content": "Beschrijf heel kort in het Nederlands wat je ziet op deze foto.", "images": [image_b64]}]
    else:
        messages = history + [{"role": "user", "content": user_input}]

    payload = {
        "model": selected_model,
        "messages": messages,
        "stream": False,
        "options": {"num_predict": 60, "temperature": 0.2}
    }
    
    try:
        response = requests.post(OLLAMA_URL, json=payload, timeout=120)
        response.raise_for_status()
        reply = response.json()['message']['content']
        
        # Alleen tekstvragen opslaan in geschiedenis om VRAM te sparen
        if not image_b64:
            history.append({"role": "user", "content": user_input})
            history.append({"role": "assistant", "content": reply})
            
        return reply, history
    except Exception as e:
        print(f"\n\033[91mFout: {e}\033[0m")
        return "Mijn systeem moet even herstarten.", history

def speak(text):
    # Verwijder triggers uit spraak
    clean = text.replace("?!snapshot", "").replace("?!ball", "").replace(EXIT_TRIGGER, "").strip()
    clean = "".join(c for c in clean if unicodedata.category(c)[0] != 'S')
    
    if clean:
        sys.stdout.write(f"\r\033[K\033[92mSara: {clean}\033[0m\n")
        temp_wav = "output.wav"
        try:
            subprocess.run(f'echo "{clean}" | {PIPER_PATH} --model {VOICE_MODEL} --output_file {temp_wav}', 
                           shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["aplay", temp_wav], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except: pass

def main():
    print("\n\033[95m--- SARA MULTIMODAL (8B/11B GPU) ---\033[0m")
    mode = input("Kies modus: [T]yping of [S]peech? ").lower()
    
    history = [{"role": "system", "content": SYSTEM_PROMPT}]
    stt_model = None
    if mode == 's':
        stt_model = WhisperModel("small", device="cpu", compute_type="int8")
    
    recognizer = sr.Recognizer()
    print(f"\n\033[96mSara is actief op {OLLAMA_URL}\033[0m")

    while True:
        user_msg = ""
        if mode == 't':
            user_msg = input("\033[93mJij: \033[0m")
        else:
            with sr.Microphone() as source:
                sys.stdout.write(f"\r\033[K\033[94mWachten op spraak...\033[0m")
                sys.stdout.flush()
                try:
                    audio = recognizer.listen(source, timeout=2, phrase_time_limit=5)
                    with open("temp.wav", "wb") as f: f.write(audio.get_wav_data())
                    segments, _ = stt_model.transcribe("temp.wav", language="nl")
                    user_msg = "".join(s.text for s in segments).strip()
                    if user_msg: print(f"\n\033[95mGehoord: {user_msg}\033[0m")
                except: continue

        if user_msg:
            # 1. Check voor afsluiten
            if any(w in user_msg.lower() for w in USER_EXIT_WORDS):
                speak("Fijne dag verder!")
                break

            # 2. Roep tekstmodel aan
            response, history = chat_with_ollama(user_msg, history)

            # 3. Check op snapshot trigger
# 3. Check op snapshot trigger
            if "?!snapshot" in response:
                # We printen dit duidelijk zodat je weet dat de camera-actie start
                print("\n\033[1;91m[CAMERA] Ik maak nu een foto...\033[0m")
                snap = get_snapshot()
                
                if snap:
                    # Belangrijk: we sturen een specifieke vraag mee naar het Vision model
                    # en we slaan het resultaat op in 'vision_reply'
                    vision_reply, history = chat_with_ollama("Beschrijf heel kort wat je ziet in de camera.", history, image_b64=snap)
                    
                    # DIT STUKJE MOET ERIN:
                    # Zorg dat Sara het antwoord ook echt geeft in de terminal en via Piper
                    print(f"\r\033[K\033[92mSara (Vision): {vision_reply}\033[0m")
                    speak(vision_reply) 
                else:
                    speak("Ik kan de camera niet openen. Controleer of een ander programma de camera gebruikt.")
            else:
                # Als er geen snapshot nodig was, heeft het tekstmodel al geantwoord
                speak(response)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgramma gestopt.")
        sys.exit(0)