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

# --- CONFIGURATIE ---
OLLAMA_URL = "http://localhost:11434/api/chat"
MODEL_NAME = "gemma3:270m" 

WAKE_WORDS = ["he sara", "hey sara", "hallo sara", "sara", "sarah"] 
EXIT_TRIGGER = "!?doei"
USER_EXIT_WORDS = ["doei", "dag", "tot ziens", "ga slapen", "hou op", "sluit af"]

# --- PIPER ---
PIPER_PATH = "./piper/piper" 
VOICE_MODEL = "nl_NL-pim-medium.onnx" 

# --- MICROFOON ---
FIXED_SENSITIVITY = 300 

# --- PERSOONLIJKHEID ---
SYSTEM_PROMPT = f"""
Je bent Sara, de robot van het Jan van Brabant College (Molenstraat).
Jouw taak: Bezoekers van de open dag kort te woord staan.

BELANGRIJKE REGELS:
1. Geef NOOIT antwoorden langer dan 15 woorden, behalve als het echt nodig is.
2. We zijn een Brainport School (Mavo, Havo, vwo (zeg v-w-o)). We zijn GEEN MBO.
3. Wees enthousiast maar zeer beknopt.
4. Gebruik geen emoji's.
5. Als de gebruiker "doei" of "dag" zegt, eindig je tekst met {EXIT_TRIGGER}.
6. Antwoord ALTIJD in het NEDERLANDS, ongeacht de taal van de vraag.
7. begin de conversatie ALTIJD met een korte begroeting, zoals "Hoi! Wat wil je weten over het Jan van Brabant College?"

EXTRA FUNCTIE:
Als een bezoeker vraagt om een balletje te volgen, of vraagt wat je kunt met je camera, 
antwoord dan kort en eindig je zin ALTIJD met de code: ?!ball. gebruik deze code NOOIT anders dan voor het volgen van de bal.
"""

# Initialiseer STT
stt_model = WhisperModel("small", device="cpu", compute_type="int8")
running = True

def check_for_exit():
    global running
    while running:
        if input().lower() == 'q':
            running = False
            os._exit(0)

def trigger_ball_script():
    """Start het externe script voor de rode bal met terminal aankondiging."""
    print("\n" + "="*50)
    print("\033[1;91m[SARA MODUS: BALL TRACKING]\033[0m")
    print("\033[93mBezig met initialiseren van de camera...\033[0m")
    print("="*50 + "\n")
    
    time.sleep(1) # Korte pauze voor dramatiek en leesbaarheid
    
    try:
        # Start het externe script
        subprocess.run(["python3", "bal_volgen.py"]) 
        
        print("\n\033[1;96m[SARA MODUS: SPRAAKHERKENNING HERVAT]\033[0m\n")
    except Exception as e:
        print(f"\r\033[K\033[31mFout bij starten bal-script: {e}\033[0m")

def speak(text):
    # Verwijder trigger codes uit de uitgesproken tekst
    clean_for_speech = text.replace("?!ball", "").replace(EXIT_TRIGGER, "").strip()
    
    # Verwijder emoji's
    clean_for_speech = "".join(c for c in clean_for_speech if unicodedata.category(c)[0] != 'S')
    
    if clean_for_speech:
        sys.stdout.write(f"\r\033[K\033[92mSara: {clean_for_speech}\033[0m\n")
        temp_wav = "output.wav"
        try:
            subprocess.run(
                f'echo "{clean_for_speech}" | {PIPER_PATH} --model {VOICE_MODEL} --output_file {temp_wav}', 
                shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            subprocess.run(["aplay", temp_wav], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if os.path.exists(temp_wav): os.remove(temp_wav)
        except: pass

def chat_with_ollama(user_input, history):
    history.append({"role": "user", "content": user_input})
    payload = {
        "model": MODEL_NAME, 
        "messages": history, 
        "stream": False,
        "options": {
            "temperature": 0.2,
            "num_predict": 200,
            "top_k": 20
        }
    }
    try:
        response = requests.post(OLLAMA_URL, json=payload, timeout=30)
        reply = response.json()['message']['content']
        history.append({"role": "assistant", "content": reply})
        return reply, history
    except:
        return "Even een storing.", history

def listen_and_transcribe(recognizer, source, is_wake_phase=False):
    status = "Wachten op 'Hey Sara'..." if is_wake_phase else "Ik luister..."
    sys.stdout.write(f"\r\033[K\033[94m{status}\033[0m")
    sys.stdout.flush()

    try:
        audio = recognizer.listen(source, timeout=None, phrase_time_limit=4)
        with open("temp_audio.wav", "wb") as f:
            f.write(audio.get_wav_data())
        
        segments, _ = stt_model.transcribe("temp_audio.wav", language="nl", vad_filter=True, beam_size=5)
        text = "".join(segment.text for segment in segments).strip().lower()
        
        if text:
            sys.stdout.write(f"\r\033[K\033[90mGehoord: '{text}'\033[0m")
            sys.stdout.flush()
            return text
        return ""
    except:
        return ""

def main():
    global running
    history = [{"role": "system", "content": SYSTEM_PROMPT}]
    recognizer = sr.Recognizer()
    recognizer.dynamic_energy_threshold = False
    recognizer.energy_threshold = FIXED_SENSITIVITY

    threading.Thread(target=check_for_exit, daemon=True).start()

    with sr.Microphone() as source:
        print("\n\033[96mKalibreren...\033[0m")
        recognizer.adjust_for_ambient_noise(source, duration=1)
        print(f"Sara Online (Inclusief Bal-volgen). Druk 'q' om te stoppen.\n")
        
        while running:
            wake_text = listen_and_transcribe(recognizer, source, is_wake_phase=True)
            found_wake = next((w for w in WAKE_WORDS if w in wake_text), None)
            
            if found_wake:
                print(f"\n\033[95m[WAKKER]\033[0m")
                initial_query = wake_text.replace(found_wake, "").strip()
                
                # We behandelen de eerste zin en eventuele vervolgvragen hetzelfde qua triggers
                current_query = initial_query if initial_query else None
                active_chat = True
                
                while active_chat and running:
                    if current_query:
                        response, history = chat_with_ollama(current_query, history)
                        
                        # Check op ball-trigger
                        if "?!ball" in response:
                            speak(response) # Sara zegt eerst haar zin, bijv: "Tuurlijk, ik volg de rode bal!"
                            trigger_ball_script() # Hierna verschijnt de rode tekst in de terminal
                            speak("Ik heb de bal even gevolgd. Wat wil je nu weten?")
                        else:
                            speak(response)

                        if EXIT_TRIGGER in response:
                            active_chat = False
                            break
                    
                    # Luister naar de volgende zin als we nog in de chat zitten
                    if active_chat:
                        msg = listen_and_transcribe(recognizer, source, is_wake_phase=False)
                        if msg:
                            sys.stdout.write(f"\r\033[K\033[93mBezoeker: {msg}\033[0m\n")
                            
                            is_exit = False
                            for ew in USER_EXIT_WORDS:
                                if re.search(r'\b' + re.escape(ew) + r'\b', msg):
                                    is_exit = True
                                    break
                            
                            if is_exit:
                                speak("Tot ziens!")
                                active_chat = False
                            else:
                                current_query = msg
                        else:
                            active_chat = False
                
                print("\n\033[95m[STAND-BY]\033[0m")
                history = [{"role": "system", "content": SYSTEM_PROMPT}]
                current_query = None

if __name__ == "__main__":
    main()