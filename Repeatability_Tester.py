import serial
import time
import csv
from pathlib import Path
from datetime import datetime

# ======================================================
# USER SETTINGS
# ======================================================

PORT = "COM3"      # Change if needed
BAUD = 115200

LEFT_POSITION_MM = 5
RIGHT_POSITION_MM = 165

DEFAULT_MICROSTEPPING = 8
DEFAULT_SPEED_MM_S = 1000
DEFAULT_ACCEL_MM_S2 = 9000

# Saves to the current Python working directory
OUTPUT_CSV = Path.cwd() / "repeatability_repeat_results100.csv"

FIELDNAMES = [
    "Date/time",
    "Requested cycles",
    "Completed cycles",
    "Left position (mm)",
    "Right position (mm)",
    "Commanded move per direction (mm)",
    "Microstepping",
    "Commanded speed (mm/s)",
    "Commanded acceleration (mm/s^2)",
    "Motion profile",
    "Calculated move time per direction (s)",
    "Total recorded move time (s)",
    "Average outward time 5_to_165 (s)",
    "Average return time 165_to_5 (s)",
    "Average outward speed (mm/s)",
    "Average return speed (mm/s)",
    "Average outward time error (%)",
    "Average return time error (%)",
    "Final internal Arduino position (mm)",
    "Measured final position (mm)",
    "Final expected position (mm)",
    "Final position error (mm)",
    "Continuous/smooth",
    "Skipped steps observed",
    "Pass/Fail/Review",
    "Observation notes",
    "Fault occurred",
    "Fault message",
]


# ======================================================
# INPUT HELPERS
# ======================================================

def ask_int(prompt, default_value):
    while True:
        value = input(prompt).strip()

        if value == "":
            return default_value

        try:
            value = int(value)
            if value > 0:
                return value
            print("Enter a positive whole number.")
        except ValueError:
            print("Enter a whole number, or press Enter for default.")


def ask_float(prompt, default_value):
    while True:
        value = input(prompt).strip()

        if value == "":
            return default_value

        try:
            return float(value)
        except ValueError:
            print("Enter a number, or press Enter for default.")


def ask_yes_no(prompt, default_value="Yes"):
    while True:
        value = input(prompt).strip().lower()

        if value == "":
            return default_value

        if value in ["y", "yes"]:
            return "Yes"

        if value in ["n", "no"]:
            return "No"

        print("Enter y/n, or press Enter for default.")


def ask_text(prompt, default_value=""):
    value = input(prompt).strip()
    return default_value if value == "" else value


# ======================================================
# SERIAL HELPERS
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


def wait_for_ready(ser, timeout=120):
    print("Waiting for Arduino startup and homing...")

    start = time.time()

    while time.time() - start < timeout:
        line = read_available_line(ser)

        if line and "Ready." in line:
            print("Arduino ready.")
            return

        time.sleep(0.01)

    raise TimeoutError("Arduino did not become ready in time.")


def wait_for_line_starting_with(ser, prefix, timeout=300):
    start = time.time()

    while time.time() - start < timeout:
        line = read_available_line(ser)

        if line and line.startswith(prefix):
            return line

        if line and "FAULT ACTIVE" in line:
            raise RuntimeError("Arduino fault occurred.")

        if line and line.startswith("REPEAT_ABORTED"):
            raise RuntimeError(line)

        if line and line.startswith("REPEAT_REJECTED"):
            raise RuntimeError(line)

        time.sleep(0.01)

    raise TimeoutError(f"Timed out waiting for line starting with: {prefix}")


def wait_for_text(ser, text, timeout=120):
    start = time.time()

    while time.time() - start < timeout:
        line = read_available_line(ser)

        if line and text in line:
            return line

        if line and "FAULT ACTIVE" in line:
            raise RuntimeError("Arduino fault occurred.")

        time.sleep(0.01)

    raise TimeoutError(f"Timed out waiting for: {text}")


def drain_serial(ser, duration=0.25):
    end_time = time.time() + duration

    while time.time() < end_time:
        line = read_available_line(ser)
        if line is None:
            time.sleep(0.01)


def home_actuator(ser):
    print("\nHoming actuator...")
    drain_serial(ser)
    send_command(ser, "HOME")
    wait_for_text(ser, "Homing complete.", timeout=120)
    print("Homing complete.")


def clear_fault_and_home(ser):
    print("\nClearing fault and homing actuator...")
    drain_serial(ser)

    send_command(ser, "CLEAR")
    time.sleep(0.5)

    drain_serial(ser)
    send_command(ser, "HOME")
    wait_for_text(ser, "Homing complete.", timeout=120)

    print("Fault cleared and actuator homed.")


# ======================================================
# REPEAT RESULT PARSING
# ======================================================

def parse_repeat_result(line):
    """
    Expected Arduino format:

    REPEAT_RESULT,
    requested_cycles,
    completed_cycles,
    left_mm,
    right_mm,
    commanded_move_mm,
    commanded_speed_mm_s,
    commanded_accel_mm_s2,
    motion_profile,
    calculated_move_time_s,
    total_recorded_move_time_s,
    average_outward_time_s,
    average_return_time_s,
    average_outward_speed_mm_s,
    average_return_speed_mm_s,
    final_internal_mm
    """

    parts = line.split(",")

    if len(parts) != 16:
        raise ValueError(f"Unexpected REPEAT_RESULT format: {line}")

    return {
        "requested_cycles": int(parts[1]),
        "completed_cycles": int(parts[2]),
        "left_mm": float(parts[3]),
        "right_mm": float(parts[4]),
        "commanded_move_mm": float(parts[5]),
        "commanded_speed_mm_s": float(parts[6]),
        "commanded_accel_mm_s2": float(parts[7]),
        "motion_profile": parts[8],
        "calculated_move_time_s": float(parts[9]),
        "total_recorded_move_time_s": float(parts[10]),
        "average_outward_time_s": float(parts[11]),
        "average_return_time_s": float(parts[12]),
        "average_outward_speed_mm_s": float(parts[13]),
        "average_return_speed_mm_s": float(parts[14]),
        "final_internal_mm": float(parts[15]),
    }


def percent_error(measured, expected):
    if expected == 0:
        return ""
    return ((measured - expected) / expected) * 100


def round_or_blank(value, digits=3):
    if value == "" or value is None:
        return ""
    return round(value, digits)


def append_row_to_csv(row):
    file_exists = OUTPUT_CSV.exists()

    with OUTPUT_CSV.open("a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=FIELDNAMES)

        if not file_exists or OUTPUT_CSV.stat().st_size == 0:
            writer.writeheader()

        writer.writerow(row)

    print(f"\nSaved CSV row to:\n{OUTPUT_CSV}")


# ======================================================
# MAIN PROGRAM
# ======================================================

def main():
    print("\nNo-Delay Arduino Repeatability Test")
    print("-----------------------------------")
    print("Python sends one REPEAT command.")
    print("Arduino performs all back-and-forth movement internally.")
    print("There is no Python delay between moves.")
    print(f"\nCurrent Python directory:\n{Path.cwd()}")
    print(f"\nCSV file location:\n{OUTPUT_CSV}")

    requested_cycles = ask_int(
        "\nEnter number of cycles [default 10]: ",
        default_value=10
    )

    microstepping = ask_int(
        f"Enter microstepping [default {DEFAULT_MICROSTEPPING}]: ",
        default_value=DEFAULT_MICROSTEPPING
    )

    speed = ask_float(
        f"Enter commanded speed in mm/s [default {DEFAULT_SPEED_MM_S}]: ",
        default_value=DEFAULT_SPEED_MM_S
    )

    accel = ask_float(
        f"Enter commanded acceleration in mm/s^2 [default {DEFAULT_ACCEL_MM_S2}]: ",
        default_value=DEFAULT_ACCEL_MM_S2
    )

    home_before_test = ask_yes_no(
        "Home actuator before starting repeatability test? [Y/n]: ",
        default_value="Yes"
    )

    fault_occurred = "No"
    fault_message = ""
    repeat_result = None

    try:
        with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
            time.sleep(2)
            ser.reset_input_buffer()

            wait_for_ready(ser)

            print("\nSending configuration...")
            send_command(ser, f"CONFIG {microstepping} {speed} {accel}")
            wait_for_line_starting_with(ser, "CONFIG_OK", timeout=20)

            if home_before_test == "Yes":
                home_actuator(ser)

            input("\nPress Enter to start REPEAT test...")

            print("\nStarting no-delay repeat test...")
            send_command(ser, f"REPEAT {requested_cycles} {LEFT_POSITION_MM} {RIGHT_POSITION_MM}")

            repeat_line = wait_for_line_starting_with(ser, "REPEAT_RESULT,", timeout=600)
            repeat_result = parse_repeat_result(repeat_line)

    except Exception as error:
        fault_occurred = "Yes"
        fault_message = str(error)

        print("\nThe repeatability test stopped due to an error:")
        print(error)

    # ======================================================
    # FINAL MANUAL RESULT ENTRY
    # ======================================================

    print("\nFinal result entry")
    print("------------------")

    if repeat_result is not None:
        completed_cycles = repeat_result["completed_cycles"]
        final_expected = LEFT_POSITION_MM

        print(f"Completed cycles: {completed_cycles} / {requested_cycles}")
        print(f"Expected final position: {final_expected} mm")
        print(f"Arduino internal final position: {repeat_result['final_internal_mm']} mm")
    else:
        completed_cycles = 0
        final_expected = LEFT_POSITION_MM
        print("No valid REPEAT_RESULT was received.")

    measured_final = ask_float(
        f"Measured final physical position in mm [default {final_expected}]: ",
        default_value=final_expected
    )

    final_position_error = measured_final - final_expected

    smooth = ask_yes_no(
        "Was the motion continuous and smooth? [Y/n]: ",
        default_value="Yes"
    )

    skipped = ask_yes_no(
        "Were skipped steps, stalls, or position loss observed? [y/N]: ",
        default_value="No"
    )

    default_result = "Pass"

    if (
        repeat_result is None
        or completed_cycles != requested_cycles
        or smooth == "No"
        or skipped == "Yes"
        or fault_occurred == "Yes"
    ):
        default_result = "Review"

    result = ask_text(
        f"Result [Pass/Fail/Review, default {default_result}]: ",
        default_value=default_result
    )

    notes = ask_text(
        "Observation notes, e.g. vibration, harsh reversal, stable, noisy: ",
        default_value=""
    )

    if repeat_result is not None:
        outward_time_error = percent_error(
            repeat_result["average_outward_time_s"],
            repeat_result["calculated_move_time_s"]
        )

        return_time_error = percent_error(
            repeat_result["average_return_time_s"],
            repeat_result["calculated_move_time_s"]
        )

        row = {
            "Date/time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "Requested cycles": repeat_result["requested_cycles"],
            "Completed cycles": repeat_result["completed_cycles"],
            "Left position (mm)": repeat_result["left_mm"],
            "Right position (mm)": repeat_result["right_mm"],
            "Commanded move per direction (mm)": repeat_result["commanded_move_mm"],
            "Microstepping": microstepping,
            "Commanded speed (mm/s)": repeat_result["commanded_speed_mm_s"],
            "Commanded acceleration (mm/s^2)": repeat_result["commanded_accel_mm_s2"],
            "Motion profile": repeat_result["motion_profile"],
            "Calculated move time per direction (s)": repeat_result["calculated_move_time_s"],
            "Total recorded move time (s)": repeat_result["total_recorded_move_time_s"],
            "Average outward time 5_to_165 (s)": repeat_result["average_outward_time_s"],
            "Average return time 165_to_5 (s)": repeat_result["average_return_time_s"],
            "Average outward speed (mm/s)": repeat_result["average_outward_speed_mm_s"],
            "Average return speed (mm/s)": repeat_result["average_return_speed_mm_s"],
            "Average outward time error (%)": round_or_blank(outward_time_error, 2),
            "Average return time error (%)": round_or_blank(return_time_error, 2),
            "Final internal Arduino position (mm)": repeat_result["final_internal_mm"],
            "Measured final position (mm)": measured_final,
            "Final expected position (mm)": final_expected,
            "Final position error (mm)": round(final_position_error, 3),
            "Continuous/smooth": smooth,
            "Skipped steps observed": skipped,
            "Pass/Fail/Review": result,
            "Observation notes": notes,
            "Fault occurred": fault_occurred,
            "Fault message": fault_message,
        }
    else:
        row = {
            "Date/time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "Requested cycles": requested_cycles,
            "Completed cycles": completed_cycles,
            "Left position (mm)": LEFT_POSITION_MM,
            "Right position (mm)": RIGHT_POSITION_MM,
            "Commanded move per direction (mm)": abs(RIGHT_POSITION_MM - LEFT_POSITION_MM),
            "Microstepping": microstepping,
            "Commanded speed (mm/s)": speed,
            "Commanded acceleration (mm/s^2)": accel,
            "Motion profile": "",
            "Calculated move time per direction (s)": "",
            "Total recorded move time (s)": "",
            "Average outward time 5_to_165 (s)": "",
            "Average return time 165_to_5 (s)": "",
            "Average outward speed (mm/s)": "",
            "Average return speed (mm/s)": "",
            "Average outward time error (%)": "",
            "Average return time error (%)": "",
            "Final internal Arduino position (mm)": "",
            "Measured final position (mm)": measured_final,
            "Final expected position (mm)": final_expected,
            "Final position error (mm)": round(final_position_error, 3),
            "Continuous/smooth": smooth,
            "Skipped steps observed": skipped,
            "Pass/Fail/Review": result,
            "Observation notes": notes,
            "Fault occurred": fault_occurred,
            "Fault message": fault_message,
        }

    append_row_to_csv(row)

    print("\nSummary recorded:")
    for key, value in row.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()