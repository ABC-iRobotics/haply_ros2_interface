#!/usr/bin/env python3

import rosbag2_py
import csv
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# --------- Configuration ----------
BAG_PATH = Path.home() / "rosbags" / "Participant0_mode2_round2"
csv_path = BAG_PATH / "rosbag_output.csv"

TOPICS_OF_INTEREST = {
    "/haply_state": "haply_msgs/msg/HaplyState",
    "/trial/state": "std_msgs/msg/String",
    "/contact_status": "std_msgs/msg/Bool",
    "/PSM1/local/measured_cp": "geometry_msgs/msg/PoseStamped",
    "/haply_target": "geometry_msgs/msg/PoseStamped",
}

RUNNING_STATES = {"RUNNING1", "RUNNING2", "RUNNING3", "RUNNING4", "RUNNING5"}
# ----------------------------------


def main():
    if not BAG_PATH.exists():
        raise FileNotFoundError(f"Rosbag nicht gefunden: {BAG_PATH}")

    storage_options = rosbag2_py.StorageOptions(
        uri=str(BAG_PATH),
        storage_id="sqlite3",
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Message types vorbereiten
    msg_types = {
        topic: get_message(msg_type)
        for topic, msg_type in TOPICS_OF_INTEREST.items()
    }

    # ---- data container ----
    time_stamps = []
    x_positions = []
    y_positions = []
    z_positions = []
    trial_states = []        
    current_trial_state = None 

    trial_running = False
    t0 = None  # Startzeitpunkt der RUNNING-Phase

    # ---- contact counter ----
    last_contact_state = False
    contact_count = 0

    print("Starte Rosbag-Auswertung...\n")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        # -------- Trial State überwachen --------
        if topic == "/trial/state":
            msg = deserialize_message(data, msg_types[topic])
            trial_running = msg.data in RUNNING_STATES
            current_trial_state = msg.data if trial_running else None

            if trial_running and t0 is None:
                t0 = timestamp  # Startzeitpunkt der ersten RUNNING-Phase
            continue


        # -------- Kontaktstatus --------
        if topic == "/contact_status":
            msg = deserialize_message(data, msg_types[topic])
            current_contact = msg.data

            if trial_running:
                # steigende Flanke: 0 -> 1
                if not last_contact_state and current_contact:
                    contact_count += 1

            last_contact_state = current_contact
            continue

        # -------- Positionsdaten nur während RUNNING --------
        if topic != "/PSM1/local/measured_cp" or not trial_running:
            continue

        msg = deserialize_message(data, msg_types[topic])

        # Zeit relativ zum RUNNING-Start
        t_sec = (timestamp - t0) * 1e-9

        time_stamps.append(t_sec)
        x_positions.append(msg.pose.position.x)
        y_positions.append(msg.pose.position.y)
        z_positions.append(msg.pose.position.z)
        trial_states.append(current_trial_state) 

    print(f"{len(x_positions)} Datapoints während RUNNING ausgewertet.")
    print(f"Anzahl Kontakte während RUNNING: {contact_count}")

    if len(x_positions) == 0:
        print("Keine Daten im RUNNING-Zeitraum gefunden.")
        return

    # ---- numpy ----
    time_stamps = np.array(time_stamps)
    x_positions = np.array(x_positions)
    y_positions = np.array(y_positions)
    z_positions = np.array(z_positions)

    mean_x = np.mean(x_positions)
    #print(f"Mittelwert x-Position (RUNNING): {mean_x:.6f} m")
    print(f"Benötigte Zeit: {time_stamps[-1] - time_stamps[0]:.2f} s")

    # ---- Plot ----
    plt.figure(figsize=(10, 4))
    plt.plot(time_stamps, x_positions, label="x")
    plt.plot(time_stamps, y_positions, label="y")
    plt.plot(time_stamps, z_positions, label="z")
    plt.xlabel("time since RUNNING start [s]")
    plt.ylabel("position [m]")
    plt.title("PSM1 position during RUNNING")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ---- CSV export ----
    csv_path = BAG_PATH / "rosbag_output.csv"
    with open(csv_path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        # Header
        writer.writerow(["time", "x", "y", "z", "contact_count", "trial_state"])
        # Daten schreiben
        for t, x, y, z, state in zip(time_stamps, x_positions, y_positions, z_positions, trial_states):
            writer.writerow([t, x, y, z, contact_count, state])

    print(f"CSV-Datei gespeichert: {csv_path}")

if __name__ == "__main__":
    main()
