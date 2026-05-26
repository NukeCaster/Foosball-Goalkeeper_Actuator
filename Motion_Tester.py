import serial
import time
import csv
from pathlib import Path

# ======================================================
# USER SETTINGS
# ======================================================

PORT = "COM3"          # Change this to your Arduino COM port
BAUD = 115200

OUTPUT_CSV = Path("Final Testing for report.csv")

# If True, actuator homes before every test.
# Usually leave False unless you are getting skipped steps often.
HOME_BEFORE_EACH_TEST = False

# If True, actuator homes automatically after a failed test.
HOME_AFTER_FAILED_TEST = True

# Automatic fail criteria
MAX_ALLOWED_POSITION_ERROR_MM = 2.0
MAX_ALLOWED_TIME_ERROR_PERCENT = 15.0

# ======================================================
# TEST LIST
# ======================================================
# Add/remove/change tests here.
# Each test means:
# current_mm -> target_mm

TESTS = [
    {"test_id": 1, "microstepping": 8, "current_mm": 5,   "target_mm": 165, "speed": 1000,  "accel": 9000},
    {"test_id": 2, "microstepping": 8, "current_mm": 165, "target_mm": 5,   "speed": 1000,  "accel": 9000},

    {"test_id": 3, "microstepping": 8, "current_mm": 5,   "target_mm": 165, "speed": 1100,  "accel": 9000},
    {"test_id": 4, "microstepping": 8, "current_mm": 165, "target_mm": 5,   "speed": 1100,  "accel": 9000}

]

# TESTS = [
#     {"test_id": 1, "microstepping": 8, "current_mm": 5,   "target_mm": 165, "speed": 250, "accel": 5000},
#     {"test_id": 2, "microstepping": 8, "current_mm": 165, "target_mm": 5,   "speed": 500, "accel": 5000},

#     {"test_id": 3, "microstepping": 8, "current_mm": 5,   "target_mm": 105, "speed": 750, "accel": 5000},
#     {"test_id": 4, "microstepping": 8, "current_mm": 105, "target_mm": 5,   "speed": 1000, "accel": 5000},

#     {"test_id": 5, "microstepping": 8, "current_mm": 5,   "target_mm": 165, "speed": 1000, "accel": 7000},
#     {"test_id": 6, "microstepping": 8, "current_mm": 165, "target_mm": 5,   "speed": 1000, "accel": 9000},

#     {"test_id": 7, "microstepping": 8, "current_mm": 5,   "target_mm": 165, "speed": 1000, "accel": 10000},
#     {"test_id": 8, "microstepping": 8, "current_mm": 165, "target_mm": 5,   "speed": 1000, "accel": 12000},

#     {"test_id": 9, "microstepping": 4, "current_mm": 5,   "target_mm": 165, "speed": 1000, "accel": 5000},
#     {"test_id": 10, "microstepping": 4, "current_mm": 165, "target_mm": 5,   "speed": 1000, "accel": 9000},

#     {"test_id": 11, "microstepping": 4, "current_mm": 5,   "target_mm": 105, "speed": 1000, "accel": 10000},
#     {"test_id": 12, "microstepping": 4, "current_mm": 105, "target_mm": 5,   "speed": 1000, "accel": 12000},

#     {"test_id": 13, "microstepping": 8, "current_mm": 5,   "target_mm": 165, "speed": 1100, "accel": 9000},
#     {"test_id": 14, "microstepping": 8, "current_mm": 165, "target_mm": 5,   "speed": 1000, "accel": 9000},

#     {"test_id": 15, "microstepping": 8, "current_mm": 5,   "target_mm": 165, "speed": 1000, "accel": 9000},
#     {"test_id": 16, "microstepping": 8, "current_mm": 165, "target_mm": 5,   "speed": 1000, "accel": 9000},
# ]

# ======================================================
# SERIAL FUNCTIONS
# ======================================================

def send_command(ser, command):
    ser.write((command + "\n").encode())
    ser.flush()
    print(f">>> {command}")


def read_available_line(ser):
    if ser.in_waiting > 0:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("Arduino:", line)
            return line
    return None


def drain_serial(ser, duration=0.25):
    end_time = time.time() + duration
    while time.time() < end_time:
        line = read_available_line(ser)
        if line is None:
            time.sleep(0.01)


def wait_for_ready(ser, timeout=90):
    print("Waiting for Arduino startup, LED flicker, and homing...")

    start = time.time()

    while time.time() - start < timeout:
        line = read_available_line(ser)

        if line and "Ready." in line:
            print("Arduino ready.")
            return

        time.sleep(0.01)

    raise TimeoutError("Arduino did not become ready in time.")


def wait_for_text(ser, text, timeout=60):
    start = time.time()

    while time.time() - start < timeout:
        line = read_available_line(ser)

        if line and text in line:
            return line

        if line and "FAULT ACTIVE" in line:
            raise RuntimeError("Arduino fault occurred.")

        time.sleep(0.01)

    raise TimeoutError(f"Timed out waiting for text: {text}")


def wait_for_line_starting_with(ser, prefix, timeout=60):
    start = time.time()

    while time.time() - start < timeout:
        line = read_available_line(ser)

        if line and line.startswith(prefix):
            return line

        if line and "FAULT ACTIVE" in line:
            raise RuntimeError("Arduino fault occurred.")

        time.sleep(0.01)

    raise TimeoutError(f"Timed out waiting for line starting with: {prefix}")


# ======================================================
# HOMING / FAULT FUNCTIONS
# ======================================================

def home_actuator(ser):
    print("\nHoming actuator...")
    drain_serial(ser)
    send_command(ser, "HOME")
    wait_for_text(ser, "Homing complete.", timeout=90)
    print("Homing complete.")


def clear_fault_and_home(ser):
    print("\nClearing fault and homing actuator...")
    drain_serial(ser)
    send_command(ser, "CLEAR")
    time.sleep(0.5)
    drain_serial(ser)
    send_command(ser, "HOME")
    wait_for_text(ser, "Homing complete.", timeout=90)
    print("Fault cleared and actuator homed.")


def show_status(ser):
    print("\nRequesting Arduino status...")
    drain_serial(ser)
    send_command(ser, "STATUS")
    time.sleep(1.0)
    drain_serial(ser, duration=1.0)


# ======================================================
# RESULT PARSING
# ======================================================

def parse_result_line(line):
    """
    Expected Arduino format:

    RESULT,
    microstepping,
    current_position_mm,
    new_commanded_position_mm,
    commanded_move_mm,
    commanded_speed_mm_s,
    commanded_accel_mm_s2,
    motion_profile,
    calculated_move_time_s,
    recorded_move_time_s,
    average_internal_speed_mm_s,
    internal_start_mm,
    internal_end_mm
    """

    parts = line.split(",")

    if len(parts) != 13:
        raise ValueError(f"Unexpected RESULT format: {line}")

    return {
        "microstepping": int(parts[1]),
        "current_position_mm": float(parts[2]),
        "new_commanded_position_mm": float(parts[3]),
        "commanded_move_mm": float(parts[4]),
        "commanded_speed_mm_s": float(parts[5]),
        "commanded_accel_mm_s2": float(parts[6]),
        "motion_profile": parts[7],
        "calculated_move_time_s": float(parts[8]),
        "recorded_move_time_s": float(parts[9]),
        "average_internal_speed_mm_s": float(parts[10]),
        "internal_start_mm": float(parts[11]),
        "internal_end_mm": float(parts[12]),
    }


# ======================================================
# CALCULATION FUNCTIONS
# ======================================================

def estimate_effective_acceleration(distance_mm, speed_mm_s, recorded_time_s, profile):
    """
    Estimates effective acceleration from the recorded move time.
    This is not direct acceleration measurement from a sensor.
    """

    s = abs(distance_mm)
    v = speed_mm_s
    t = recorded_time_s

    if s <= 0 or v <= 0 or t <= 0:
        return None

    profile = profile.upper()

    if profile == "TRIANGULAR":
        return (4 * s) / (t ** 2)

    if profile == "TRAPEZOIDAL":
        denominator = t - (s / v)

        if denominator <= 0:
            return None

        return v / denominator

    return None


def percent_error(measured, expected):
    if measured is None or expected == 0:
        return None

    return ((measured - expected) / expected) * 100


def ask_float(prompt, default_value):
    while True:
        user_input = input(prompt).strip()

        if user_input == "":
            return default_value

        try:
            return float(user_input)
        except ValueError:
            print("Enter a number, or press Enter to use the default.")


# ======================================================
# USER MENUS
# ======================================================

def pre_test_menu(ser, test_id):
    while True:
        choice = input(
            f"\nBefore Test {test_id}: "
            "[Enter] run, [h] home, [fh] clear fault + home, "
            "[status] status, [s] skip, [q] quit: "
        ).strip().lower()

        if choice == "":
            return "run"

        if choice == "h":
            home_actuator(ser)

        elif choice == "fh":
            clear_fault_and_home(ser)

        elif choice == "status":
            show_status(ser)

        elif choice == "s":
            return "skip"

        elif choice == "q":
            return "quit"

        else:
            print("Unknown option.")


def post_test_menu(ser):
    while True:
        choice = input(
            "\nAfter test: "
            "[Enter] continue, [h] home, [fh] clear fault + home, "
            "[status] status, [q] quit: "
        ).strip().lower()

        if choice == "":
            return "continue"

        if choice == "h":
            home_actuator(ser)

        elif choice == "fh":
            clear_fault_and_home(ser)

        elif choice == "status":
            show_status(ser)

        elif choice == "q":
            return "quit"

        else:
            print("Unknown option.")


# ======================================================
# SINGLE TEST RUNNER
# ======================================================

def run_single_test(ser, test):
    test_id = test["test_id"]
    current_mm = test["current_mm"]
    target_mm = test["target_mm"]
    commanded_move = abs(target_mm - current_mm)

    print("\n" + "=" * 70)
    print(f"TEST {test_id}")
    print(f"Current/start position: {current_mm} mm")
    print(f"New commanded position: {target_mm} mm")
    print(f"Commanded move: {commanded_move} mm")
    print(f"Commanded speed: {test['speed']} mm/s")
    print(f"Commanded acceleration: {test['accel']} mm/s^2")
    print("=" * 70)

    if HOME_BEFORE_EACH_TEST:
        home_actuator(ser)

    drain_serial(ser)
    send_command(ser, f"TEST {current_mm} {target_mm}")

    result_line = wait_for_line_starting_with(ser, "RESULT,", timeout=90)
    result = parse_result_line(result_line)

    print("\nManual measurement section:")
    print("Measure physical start and final position from the same reference point.")
    print("Press Enter to accept the commanded value if you did not measure it.")

    measured_start = ask_float(
        f"Measured start position in mm [default {current_mm:.2f}]: ",
        current_mm
    )

    measured_final = ask_float(
        f"Measured final position in mm [default {target_mm:.2f}]: ",
        target_mm
    )

    measured_travel = abs(measured_final - measured_start)

    travel_error = measured_travel - commanded_move
    position_error = measured_final - target_mm

    recorded_time = result["recorded_move_time_s"]

    if recorded_time > 0:
        average_recorded_speed = measured_travel / recorded_time
    else:
        average_recorded_speed = None

    time_error = percent_error(
        result["recorded_move_time_s"],
        result["calculated_move_time_s"]
    )

    estimated_accel = estimate_effective_acceleration(
        result["commanded_move_mm"],
        result["commanded_speed_mm_s"],
        result["recorded_move_time_s"],
        result["motion_profile"]
    )

    accel_error = percent_error(
        estimated_accel,
        result["commanded_accel_mm_s2"]
    )

    observation = input(
        "Motion observation, e.g. smooth/vibration/skipped steps/harsh: "
    ).strip()

    result_text = input(
        "Result, e.g. Pass/Fail/Review: "
    ).strip()

    observation_lower = observation.lower()
    result_lower = result_text.lower()

    manual_fail = result_lower.startswith("f")

    observed_step_loss = (
        "skip" in observation_lower
        or "missed" in observation_lower
        or "lost" in observation_lower
        or "stall" in observation_lower
    )

    position_error_fail = abs(position_error) > MAX_ALLOWED_POSITION_ERROR_MM

    time_error_fail = False
    if time_error is not None:
        time_error_fail = abs(time_error) > MAX_ALLOWED_TIME_ERROR_PERCENT

    test_failed = (
        manual_fail
        or observed_step_loss
        or position_error_fail
        or time_error_fail
    )

    row = {
        "Test ID": test_id,
        "Microstepping": result["microstepping"],
        "Current position (mm)": round(result["current_position_mm"], 3),
        "New commanded position (mm)": round(result["new_commanded_position_mm"], 3),
        "Commanded move (mm)": round(result["commanded_move_mm"], 3),
        "Commanded speed (mm/s)": round(result["commanded_speed_mm_s"], 3),
        "Commanded acceleration (mm/s^2)": round(result["commanded_accel_mm_s2"], 3),
        "Motion profile": result["motion_profile"],
        "Calculated move time (s)": round(result["calculated_move_time_s"], 4),
        "Recorded move time (s)": round(result["recorded_move_time_s"], 4),
        "Time error (%)": "" if time_error is None else round(time_error, 2),
        "Estimated effective acceleration (mm/s^2)": "" if estimated_accel is None else round(estimated_accel, 1),
        "Acceleration error (%)": "" if accel_error is None else round(accel_error, 2),
        "Average recorded speed (mm/s)": "" if average_recorded_speed is None else round(average_recorded_speed, 3),
        "Measured start position (mm)": round(measured_start, 3),
        "Measured final position (mm)": round(measured_final, 3),
        "Measured travel (mm)": round(measured_travel, 3),
        "Travel error (mm)": round(travel_error, 3),
        "Position error (mm)": round(position_error, 3),
        "Motion observation": observation,
        "Result": result_text,
        "Auto fail detected": "Yes" if test_failed else "No",
    }

    print("\nRecorded row:")
    for key, value in row.items():
        print(f"{key}: {value}")

    if HOME_AFTER_FAILED_TEST and test_failed:
        print("\nTEST MARKED AS FAILED OR OUTSIDE LIMITS.")
        print("The actuator will be re-homed before continuing.")
        home_actuator(ser)

    return row


# ======================================================
# SAVE RESULTS
# ======================================================

def save_results(rows):
    fieldnames = [
        "Test ID",
        "Microstepping",
        "Current position (mm)",
        "New commanded position (mm)",
        "Commanded move (mm)",
        "Commanded speed (mm/s)",
        "Commanded acceleration (mm/s^2)",
        "Motion profile",
        "Calculated move time (s)",
        "Recorded move time (s)",
        "Time error (%)",
        "Estimated effective acceleration (mm/s^2)",
        "Acceleration error (%)",
        "Average recorded speed (mm/s)",
        "Measured start position (mm)",
        "Measured final position (mm)",
        "Measured travel (mm)",
        "Travel error (mm)",
        "Position error (mm)",
        "Motion observation",
        "Result",
        "Auto fail detected",
    ]

    with OUTPUT_CSV.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print("\nDone.")
    print(f"Saved results to: {OUTPUT_CSV.resolve()}")


# ======================================================
# MAIN PROGRAM
# ======================================================

def main():
    rows = []
    last_microstepping = None

    try:
        with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
            time.sleep(2)
            ser.reset_input_buffer()

            wait_for_ready(ser)

            for test in TESTS:
                test_id = test["test_id"]
                microstepping = test["microstepping"]
                speed = test["speed"]
                accel = test["accel"]

                print("\n" + "#" * 70)
                print(f"Preparing Test {test_id}")
                print("#" * 70)

                if microstepping != last_microstepping:
                    print("\nIMPORTANT:")
                    print(f"Set the TB6600 DIP switches to microstepping = {microstepping}.")
                    print("The Arduino CONFIG value must match the physical TB6600 DIP switch setting.")
                    input("Press Enter after the DIP switches are set correctly...")

                    drain_serial(ser)
                    send_command(ser, f"CONFIG {microstepping} {speed} {accel}")
                    wait_for_line_starting_with(ser, "CONFIG_OK", timeout=15)

                    print("Re-homing after microstepping change...")
                    home_actuator(ser)

                    last_microstepping = microstepping

                else:
                    drain_serial(ser)
                    send_command(ser, f"CONFIG {microstepping} {speed} {accel}")
                    wait_for_line_starting_with(ser, "CONFIG_OK", timeout=15)

                menu_result = pre_test_menu(ser, test_id)

                if menu_result == "skip":
                    print(f"Skipping Test {test_id}.")
                    continue

                if menu_result == "quit":
                    print("Quitting test program.")
                    break

                try:
                    row = run_single_test(ser, test)
                    rows.append(row)

                except RuntimeError as error:
                    print(f"\nTest {test_id} stopped because of an Arduino fault:")
                    print(error)

                    try:
                        clear_fault_and_home(ser)
                    except Exception as recovery_error:
                        print("Could not recover automatically:")
                        print(recovery_error)

                    fault_row = {
                        "Test ID": test_id,
                        "Microstepping": microstepping,
                        "Current position (mm)": test["current_mm"],
                        "New commanded position (mm)": test["target_mm"],
                        "Commanded move (mm)": abs(test["target_mm"] - test["current_mm"]),
                        "Commanded speed (mm/s)": speed,
                        "Commanded acceleration (mm/s^2)": accel,
                        "Motion profile": "",
                        "Calculated move time (s)": "",
                        "Recorded move time (s)": "",
                        "Time error (%)": "",
                        "Estimated effective acceleration (mm/s^2)": "",
                        "Acceleration error (%)": "",
                        "Average recorded speed (mm/s)": "",
                        "Measured start position (mm)": "",
                        "Measured final position (mm)": "",
                        "Measured travel (mm)": "",
                        "Travel error (mm)": "",
                        "Position error (mm)": "",
                        "Motion observation": f"Arduino fault: {error}",
                        "Result": "Fail",
                        "Auto fail detected": "Yes",
                    }

                    rows.append(fault_row)
                    continue

                except TimeoutError as error:
                    print(f"\nTest {test_id} timed out:")
                    print(error)

                    try:
                        clear_fault_and_home(ser)
                    except Exception as recovery_error:
                        print("Could not recover automatically:")
                        print(recovery_error)
                        break

                    continue

                post_result = post_test_menu(ser)

                if post_result == "quit":
                    print("Quitting test program.")
                    break

    finally:
        save_results(rows)


if __name__ == "__main__":
    main()