from pynput import keyboard
def on_press(k): print("press", k)
def on_release(k): print("rel", k)
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()
input("press enter to quit\n")
listener.stop()
