import serial
import time

PORT = "COM3"      # Change this to your Arduino port
BAUD = 115200

test_positions = [
    20,
    165,
    10,
    155,
    10,
    145,
    20,
    135,
    30,
    125,
    40,
    115,
    50,
    105,
    60,
    95,
    70,
    85,
    80,
    75,
    90,
    65,
    100,
    55,
    110,
    45,
    120,
    35,
    130,
    25,
    140,
    15,
    150,
    5,
    160,
    5
]

# test_positions = [
#     5,
#     165, 
#     5, 
#     165,
#     5,
#     165, 
#     5, 
#     165,
#     5, 
#     165,
#     5, 
#     165
# ]





def send_command(ser, command):
    ser.write((str(command) + "\n").encode())
    print(f"Sent: {command}")

def read_available(ser):
    lines = []
    while ser.in_waiting > 0:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("Arduino:", line)
            lines.append(line)
    return lines

def wait_for_ready(ser, timeout=20):
    print("Waiting for Arduino to finish startup and homing...")

    start_time = time.time()

    while time.time() - start_time < timeout:
        lines = read_available(ser)

        for line in lines:
            if "Ready" in line:
                print("Arduino is ready.")
                return True

        time.sleep(0.05)

    print("Timed out waiting for Ready.")
    return False

ser = serial.Serial(PORT, BAUD, timeout=0.1)

try:
    print("Connected to Arduino.")
    print("Arduino will reset, wait, and home automatically.")

    if not wait_for_ready(ser, timeout=25):
        print("Arduino did not report Ready. Sending STATUS.")
        send_command(ser, "STATUS")
        time.sleep(1)
        read_available(ser)
        raise RuntimeError("Arduino not ready.")

    print("\nStarting movement test...")

    prev_pos = None
    min_delay = 0.05
    max_delay = 0.3
    max_delta = 165

    # delay = 0.2

    for pos in test_positions:
        send_command(ser, pos)

        if prev_pos is None:
            delay = max_delay
        else:
            delta = abs(pos - prev_pos)
            delay = min_delay + (max_delay - min_delay) * (delta / max_delta)
            print(f"Delay received: {delay} seconds")
            if delay < min_delay:
                delay = min_delay
            if delay > max_delay:
                delay = max_delay
        
        start_time = time.time()
        while time.time() - start_time < delay:
            read_available(ser)
            time.sleep(0.05)

        prev_pos = pos

    # send_command(ser, -20)   # Should clamp to 5
    # time.sleep(1.5)
    # read_available(ser)

    # send_command(ser, 250)   # Should clamp to 175
    # time.sleep(1.5)
    # read_available(ser)

    print("\nReturning to centre...")
    # send_command(ser, 90)
    # time.sleep(1.5)
    # read_available(ser)

    print("\nTest complete.")

except KeyboardInterrupt:
    print("\nKeyboard interrupt. Sending STOP.")
    send_command(ser, "STOP")
    time.sleep(0.5)
    read_available(ser)

finally:
    ser.close()
    print("Serial closed.")