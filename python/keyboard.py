import serial
import time
import threading
import pynput.keyboard as keyboard

# Initialize serial communication
# Adjust the port and baud rate as needed
ser = serial.Serial('/dev/cu.usbserial-210', 115200)


# Initialize coordinates
x, y, z = 0.1, 0, 0

keys = {"w": False, "a": False, "s": False, "d": False, "q": False, "e": False, "z": False}


def on_key_press(key):
    global keys
    try:
        keys[key.char] = True
    except AttributeError:
        pass


def on_key_release(key):
    global keys
    try:
        keys[key.char] = False
    except AttributeError:
        pass


def send_coordinates():
    global x, y, z
    while True:
        if keys["w"]:
            x += 0.001
        if keys["s"]:
            x -= 0.001
        if keys["a"]:
            y -= 0.001
        if keys["d"]:
            y += 0.001
        if keys["q"]:
            z += 0.001
        if keys["e"]:
            z -= 0.001
        if keys["z"]:
            break

        ser.write(f"{x},{y},{z}\n".encode())
        time.sleep(0.1)


# create a listener and setup our call backs
keyboard_listener = keyboard.Listener(
    on_press=on_key_press,
    on_release=on_key_release)

# start the listener
print("starting the keyboard listener, will run until the program is stopped...")
keyboard_listener.start()

# start the thread to send coordinates
thread = threading.Thread(target=send_coordinates)
thread.start()

# wait for the thread to finish
thread.join()
