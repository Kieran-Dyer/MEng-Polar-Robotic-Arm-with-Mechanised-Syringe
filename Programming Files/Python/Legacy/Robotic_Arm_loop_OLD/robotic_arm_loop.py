import serial
import time

# Define the serial port and baud rate (Adjust the port as needed)
SERIAL_PORT = "/dev/ttyACM0"  # Change to your Arduino's port (e.g., /dev/ttyACM0)
BAUD_RATE = 9600

ser = None

# Predefined Cartesian coordinates
coordinates = {
    "Zero": (0, 0, 0),
    "P1": (100, -100, 0),
    "P2": (100, 0, 0),
    "P3": (100, 100, 0),
}

# Open the serial connection
while ser is None:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        ser.flushInput()
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Serial connection failed: {e}")
        print(f"Retrying in 1 second ...")
        time.sleep(1)

def send_command(command):
    """Send a command to Arduino and wait for a response."""
    ser.write((command + "\n").encode())  # Send command
    time.sleep(0.1)  # Small delay before reading response

    while True:
        response = ser.readline().decode().strip()  # Read response
        if response:
            print(f"Arduino: {response}")
        if "arm movement complete" in response.lower():
            break  # Proceed to the next command

# Loop through predefined positions
while True:
    for name, (x, y, z) in coordinates.items():
        cmd = f"goto ({x}, {y}, {z})"
        print(f"Sending: {cmd}")
        send_command(cmd)
        time.sleep(0.5)  # Wait between commands

