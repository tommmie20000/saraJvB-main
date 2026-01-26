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
OLLAMA_URL = "https://localhost:11434/api/chat" 
TEXT_MODEL = "gemma3:270m"         
VISION_MODEL = "llama3.2-vision:11b" 
MEMORY_FILE = "AI/sara_memories.txt"

WAKE_WORDS = ["he sara", "hey sara", "hallo sara", "sara", "sarah"] 
EXIT_TRIGGER = "!?doei"
USER_EXIT_WORDS = ["doei", "dag", "tot ziens", "hou op", "stop"]

# --- PIPER ---
PIPER_PATH = "piper/piper" 
VOICE_MODEL = "nl_NL-pim-medium.onnx" 

# --- GEHEUGEN FUNCTIES ---
def read_memories():
    """Leest de laatste 5 herinneringen uit het bestand."""
    if not os.path.exists(MEMORY_FILE):
        return "Je hebt nog geen eerdere bezoekers gesproken vandaag."
    with open(MEMORY_FILE, "r") as f:
        lines = f.readlines()
        # Pak de laatste 5 herinneringen om de prompt niet te zwaar te maken
        return "".join(lines[-5:])

def save_memory(summary):
    """Slaat een nieuwe herinnering op."""
    with open(MEMORY_FILE, "a") as f:
        f.write(f"- {summary}\n")

def summarize_conversation(history):
    """Vraagt de AI om een ultrakorte samenvatting van het gesprek."""
    # We filteren de systeem-prompt er even uit voor de samenvatting
    clean_history = [m for m in history if m['role'] != 'system']
    
    prompt = "Vat dit gesprek samen in maximaal 10 woorden voor je geheugen. Gebruik de derde persoon. Voorbeeld: 'Een bezoeker vroeg naar de v-w-o lokalen.'"
    
    payload = {
        "model": TEXT_MODEL,
        "messages": clean_history + [{"role": "user", "content": prompt}],
        "stream": False,
        "options": {"num_predict": 30}
    }
    try:
        response = requests.post(OLLAMA_URL, json=payload, timeout=30)
        return response.json()['message']['content'].strip()
    except:
        return None

# --- CORE FUNCTIES ---
def get_snapshot():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened(): return None
    time.sleep(0.5)
    success, frame = cap.read()
    if success:
        frame = cv2.resize(frame, (640, 480))
        _, buffer = cv2.imencode('.jpg', frame)
        img_str = base64.b64encode(buffer).decode('utf-8')
        cap.release()
        return img_str
    cap.release()
    return None

def chat_with_ollama(user_input, history, image_b64=None):
    selected_model = VISION_MODEL if image_b64 else TEXT_MODEL
    
    if image_b64:
        messages = [{"role": "user", "content": "Beschrijf heel kort in het Nederlands wat je ziet op deze foto.", "images": [image_b64]}]
    else:
        messages = history + [{"role": "user", "content": user_input}]

    payload = {
        "model": selected_model,
        "messages": messages,
        "stream": False,
        "options": {"num_predict": 80, "temperature": 0.4}
    }
    
    try:
        response = requests.post(OLLAMA_URL, json=payload, timeout=120)
        reply = response.json()['message']['content']
        if not image_b64:
            history.append({"role": "user", "content": user_input})
            history.append({"role": "assistant", "content": reply})
        return reply, history
    except Exception as e:
        print(f"\n\033[91mFout: {e}\033[0m")
        return "Systeemfout.", history

def speak(text):
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
    print("\n\033[95m--- SARA MULTIMODAL MET GEHEUGEN ---\033[0m")
    mode = input("Kies modus: [T]yping of [S]peech? ").lower()
    
    # LADEN VAN GEHEUGEN
    memories = read_memories()
    
    system_prompt = f"""
    Je bent Sara, de robot van het Jan van Brabant College. 
    Korte antwoorden (max 15 woorden). 
    Probeer zo menselijk mogelijk te klinken, dus begin bijvoorbeeld zinnen op een natuurlijke wijze.
    Neem geen dingen aan die niet waar zijn.
    Als iemand afscheid neemt, zeg dan altijd: "!?doei" om het gesprek netjes af te sluiten.
    GEHEUGEN VAN VANDAAG:
    {memories}
    
    Gebruik dit geheugen af en toe om te laten zien dat je mensen onthoudt. Door bijvoorbeeld te zeggen: "Ik kan me herinneren dat hier eerder ook iemand was die ...".
    """
    
    history = [{"role": "system", "content": system_prompt}]
    stt_model = WhisperModel("small", device="cpu", compute_type="int8") if mode == 's' else None
    recognizer = sr.Recognizer()

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
            # 1. Check voor afsluiten en herinnering opslaan
            if any(w in user_msg.lower() for w in USER_EXIT_WORDS):
                speak("Fijne dag! Ik zal je onthouden.")
                print("\033[90mGeheugen aan het opslaan...\033[0m")
                summary = summarize_conversation(history)
                if summary:
                    save_memory(summary)
                break

            # 2. Roep model aan
            response, history = chat_with_ollama(user_msg, history)

            # 3. Snapshot logica
            if "?!snapshot" in response:
                print("\n\033[1;91m[CAMERA] Foto maken...\033[0m")
                snap = get_snapshot()
                if snap:
                    # Vision aanroep (zonder history om loops te voorkomen)
                    vision_reply, _ = chat_with_ollama("Beschrijf kort wat je ziet.", [], image_b64=snap)
                    clean_vision = vision_reply.replace("?!snapshot", "").strip()
                    print(f"\r\033[K\033[92mSara ziet: {clean_vision}\033[0m")
                    speak(clean_vision)
                    history.append({"role": "assistant", "content": f"Ik zie op de camera: {clean_vision}"})
                else:
                    speak("Ik zie momenteel niets.")
            else:
                speak(response)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)