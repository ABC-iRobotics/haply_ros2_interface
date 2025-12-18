#!/usr/bin/env python3

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# --------- Configuration ----------
BAG_PATH = Path.home() / "rosbags" / "Participant0_mode1_round1"

TOPICS_OF_INTEREST = {
    "/haply_state": "haply_msgs/msg/HaplyState",
    "/contact_status": "std_msgs/msg/Bool",
    "/PSM1/local/measured_cp": "geometry_msgs/msg/PoseStamped",
}
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

    # prepare message-types
    msg_types = {
        topic: get_message(msg_type)
        for topic, msg_type in TOPICS_OF_INTEREST.items()
    }

    # ---- data container ----
    time_stamps = []
    x_positions = []
    y_positions = []
    z_positions = []
    t0 = None  # relative start time


    print("Starte Rosbag-Auswertung...\n")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic != "/PSM1/local/measured_cp":
            continue

        msg = deserialize_message(data, msg_types[topic])

        if t0 is None:
            t0 = timestamp

        # time in seconds (relative to start)
        t_sec = (timestamp - t0) * 1e-9

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        time_stamps.append(t_sec)
        x_positions.append(x)
        y_positions.append(y)
        z_positions.append(z)   

    print(f"{len(x_positions)} datapoints read.")

    # Convert to numpy arrays
    time_stamps = np.array(time_stamps)
    x_positions = np.array(x_positions)
    y_positions = np.array(y_positions)
    z_positions = np.array(z_positions)

    mean_x = np.mean(x_positions)
    print(f"Mittelwert x-Position: {mean_x:.6f} m")

    # ---- Plot ----
    plt.figure(figsize=(10, 4))
    plt.plot(time_stamps, x_positions, label="x")
    plt.plot(time_stamps, y_positions, label="y")
    plt.plot(time_stamps, z_positions, label="z")
    plt.xlabel("time [s]")
    plt.ylabel("position [m]")
    plt.title("PSM1 position over time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
