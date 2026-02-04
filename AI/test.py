import wave, json, audioop
from vosk import Model, KaldiRecognizer

wf = wave.open("test.wav","rb")
raw = wf.readframes(wf.getnframes())
sampwidth = wf.getsampwidth()
channels = wf.getnchannels()
rate = wf.getframerate()

# stereo -> mono
if channels == 2:
    raw = audioop.tomono(raw, sampwidth, 1, 1)

# resample to 16000 if needed
if rate != 16000:
    raw, _ = audioop.ratecv(raw, sampwidth, 1, rate, 16000, None)

model = Model("models/vosk-model-small-nl-0.22")
rec = KaldiRecognizer(model, 16000, json.dumps(["sara"]))
rec.AcceptWaveform(raw)
print(rec.Result())