import serial
import time
import sys
import threading
import queue

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 9600

ser = None
input_q = queue.Queue()

# Cartesian lookup
coordinates = {
    "Zero": (0, 0, 0),
    "P1":   (100, -100, 0),
    "P2":   (100, 0, 0),
    "P3":   (100, 100, 0),
}

# Automatic program
program = [
    "goto P1",
    "draw syringe 100",
    "goto P2",
    "inject syringe 100",
    "goto Zero",
]

# Connect to Arduino
while ser is None:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        ser.flushInput()
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Serial connection failed: {e}")
        time.sleep(1)

def expand_goto(cmd: str) -> str:
    parts = cmd.strip().split()
    if len(parts) == 2 and parts[0].lower() == "goto" and parts[1] in coordinates:
        x, y, z = coordinates[parts[1]]
        return f"goto ({x}, {y}, {z})"
    return cmd

def send_command(command: str):
    ser.write((command + "\n").encode())

def serial_reader():
    """Background thread to print any Arduino output immediately."""
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print(f"\nArduino: {line}")
                # Prompt again if user is typing
                print("Manual> ", end="", flush=True)
        except Exception:
            break

def input_reader():
    """Background thread to collect keyboard input without blocking."""
    while True:
        try:
            text = input("Manual> ").strip()
            input_q.put(text)
        except EOFError:
            break

def manual_loop():
    while True:
        cmd = input_q.get()  # waits for user to type something

        if not cmd:
            continue

        low = cmd.lower()

        if cmd == "!":
            print("Sending emergency stop.")
            send_command("!")
            continue

        if low in {"automate", "start loop"}:
            return "auto"

        if low in {"quit", "exit"}:
            raise KeyboardInterrupt

        send_command(expand_goto(cmd))

def automatic_loop():
    for step in program:
        print(f"Auto sending: {step}")
        send_command(expand_goto(step))

        # Wait until Arduino reports any "...complete" line
        while True:
            try:
                # Check if user typed something
                if not input_q.empty():
                    cmd = input_q.get().strip()
                    if cmd == "!":
                        print("Emergency stop from user.")
                        send_command("!")
                        return
                    if cmd.lower() in {"stop", "exit loop"}:
                        print("Stopping automatic loop.")
                        return
                # Peek at Arduino output
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    print(f"Arduino: {line}")
                    if "complete" in line.lower():
                        break
            except Exception:
                return

# --- Start background threads for continuous input and Arduino printing ---
threading.Thread(target=serial_reader, daemon=True).start()
threading.Thread(target=input_reader, daemon=True).start()

try:
    mode = "manual"
    while True:
        if mode == "manual":
            mode = manual_loop()
        else:
            automatic_loop()
            mode = "manual"
except KeyboardInterrupt:
    print("\nExiting cleanly...")
    if ser and ser.is_open:
        ser.close()
    sys.exit(0)
