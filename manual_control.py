import serial
import time
from pynput import keyboard  # For listening to keyboard inputs

# Initialize connection with the Arduino
serial_port = '/dev/tty.usbserial-110'  # Change to your Arduino's port
baud_rate = 9600  # Match the baud rate of the Arduino
arduino = serial.Serial(serial_port, baud_rate, timeout=1)
time.sleep(2)  # Allow time for Arduino to reset

# Initialize light parametersdaw
light_0 = {'pan': 170, 'tilt': 200, 'r': 255, 'g': 255, 'b': 255, 'w': 255, 'm': 0}
light_1 = {'pan': 100, 'tilt': 50, 'r': 255, 'g': 0, 'b': 0, 'w': 255, 'm': 0}

# Calibration limits
light_0_limits = {'top': 255, 'bottom': 155, 'left': 140, 'right': 200}
light_1_limits = {'top': 0, 'bottom': 100, 'left': 78, 'right': 123}

# Function to update light values and send data to Arduino
def send_to_arduino():
    msg = (
        f"0:{light_0['pan']},{light_0['tilt']},{light_0['r']},{light_0['g']},{light_0['b']},{light_0['w']},{light_0['m']};"
        f"1:{light_1['pan']},{light_1['tilt']},{light_1['r']},{light_1['g']},{light_1['b']},{light_1['w']},{light_1['m']};\n"
    )
    print(f"Sending: {msg}")
    arduino.write(msg.encode())

# Key presses to adjust lights
def on_press(key):
    global light_0, light_1
    try:
        # For light 0: WASD keys
        if key.char in ['w', 'a', 's', 'd']:
            if key.char == 'w':  # tilt up
                light_0['tilt'] = min(light_0['tilt'] + 2, light_0_limits['top'])
            elif key.char == 's':  # tilt down
                light_0['tilt'] = max(light_0['tilt'] - 2, light_0_limits['bottom'])
            elif key.char == 'a':  # pan right
                light_0['pan'] = min(light_0['pan'] + 2, light_0_limits['right'])
            elif key.char == 'd':  # pan left
                light_0['pan'] = max(light_0['pan'] - 2, light_0_limits['left'])
            send_to_arduino()
    except AttributeError:
        # For light 1: Arrow keys
        if key == keyboard.Key.up:  # tilt up
            light_1['tilt'] = min(light_1['tilt'] + 2, light_1_limits['bottom'])
        elif key == keyboard.Key.down:  # tilt down
            light_1['tilt'] = max(light_1['tilt'] - 2, light_1_limits['top'])
        elif key == keyboard.Key.right:  # pan right
            light_1['pan'] = min(light_1['pan'] + 2, light_1_limits['right'])
        elif key == keyboard.Key.left:  # pan left
            light_1['pan'] = max(light_1['pan'] - 2, light_1_limits['left'])
        send_to_arduino()

# Start listening to keyboard events
def main():
    print("Use W, A, S, D keys for Light 0 and Arrow keys for Light 1.")
    print("Press Esc to exit.")
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()  # Wait for events (blocks until Esc is pressed)
    arduino.close()

# Run the application
if __name__ == "__main__":
    main()