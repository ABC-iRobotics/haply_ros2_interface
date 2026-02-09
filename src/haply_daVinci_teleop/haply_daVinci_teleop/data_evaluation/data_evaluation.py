#!/usr/bin/env python3

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# --------- Configuration ----------
BAG_PATH = Path.home() / "rosbags" / "Participant0_mode2_round3"
csv_path = BAG_PATH / "rosbag_output.csv"

TOPICS_OF_INTEREST = {
    "/haply_state": "haply_msgs/msg/HaplyState",
    "/trial/state": "std_msgs/msg/String",
    "/contact_status": "std_msgs/msg/Bool",
    "/PSM1/local/measured_cp": "geometry_msgs/msg/PoseStamped",
    "/haply_target": "haply_msgs/msg/HaplyControl",
    "/loop_center": "geometry_msgs/msg/PointStamped",
}

RUNNING_STATES = {"RUNNING1", "RUNNING2", "RUNNING3", "RUNNING4", "RUNNING5"}
# ----------------------------------
def point_to_segment_distance(p, a, b):
    """
    Computes the shortest distance between point p and line segment ab.
    """
    ab = b - a
    ap = p - a

    denom = np.dot(ab, ab)
    if denom == 0.0:
        return np.linalg.norm(ap)

    t = np.dot(ap, ab) / denom
    t = np.clip(t, 0.0, 1.0)

    closest = a + t * ab
    return np.linalg.norm(p - closest)

# ------------------ Main Evaluation Function -----------------
def main():
    if not BAG_PATH.exists():
        raise FileNotFoundError(f"Rosbag not found: {BAG_PATH}")

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

    # prepare message types
    msg_types = {
        topic: get_message(msg_type)
        for topic, msg_type in TOPICS_OF_INTEREST.items()
    }

    # ---- data container ----
    time_stamps = []
    x_positions = []
    y_positions = []
    z_positions = []

    loop_time_stamps = []
    loop_centers = []
    trial_states = []     
    # ---- force data container ----
    force_time_stamps = []
    fx_values = []
    fy_values = []
    fz_values = []
    # ---- state container ----
    state_change_times = []
    state_change_labels = []
    last_state = None
    # ---- wire points ----
    given_wirepoints = np.array([
        [-0.05, 0.05, -0.2],
        [-0.05, 0.05, -0.16],
        [-0.02, 0.05, -0.16],
        [-0.02, 0.1, -0.16],
        [0.03, 0.1, -0.16],
        [0.03, 0.1, -0.2],
    ])
    state_to_segment = {
        "RUNNING1": (0, 1),
        "RUNNING2": (1, 2),
        "RUNNING3": (2, 3),
        "RUNNING4": (3, 4),
        "RUNNING5": (4, 5),
    }
   
    current_trial_state = None 
    trial_running = False
    t0 = None  # Start of RUNNING phase

    # ---- contact counter ----
    last_contact_state = False
    contact_count = 0

    print("Start Rosbag evaluation...\n")

    # -------- Iterate through rosbag messages --------
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        # -------- Check Trial States --------
        if topic == "/trial/state":
            msg = deserialize_message(data, msg_types[topic])

            new_state = msg.data
            trial_running = msg.data in RUNNING_STATES
            current_trial_state = msg.data if trial_running else None

            if trial_running and t0 is None:
                t0 = timestamp  # Startpoint for time measurement
            
            # ---- detect state change ----
            if t0 is not None and new_state != last_state:
                t_state = (timestamp - t0) * 1e-9
                state_change_times.append(t_state)
                state_change_labels.append(new_state)

            last_state = new_state
            continue


        # -------- Contact status --------
        if topic == "/contact_status":
            msg = deserialize_message(data, msg_types[topic])
            current_contact = msg.data

            if trial_running:
                # only count transitions from no contact to contact during RUNNING
                if not last_contact_state and current_contact:
                    contact_count += 1

            last_contact_state = current_contact
            continue

        # -------- Haply forces --------
        if topic == "/haply_target" and trial_running:
            msg = deserialize_message(data, msg_types[topic])

            t_force = (timestamp - t0) * 1e-9

            force_time_stamps.append(t_force)
            fx_values.append(msg.force.x)
            fy_values.append(msg.force.y)
            fz_values.append(msg.force.z)
            continue

        # -------- Loop center --------
        if topic == "/loop_center" and trial_running:
            msg = deserialize_message(data, msg_types[topic])

            t_loop = (timestamp - t0) * 1e-9

            loop_time_stamps.append(t_loop)
            loop_centers.append([msg.point.x, msg.point.y, msg.point.z])
            trial_states.append(current_trial_state)
            continue

        # -------- Tool position --------
        if topic != "/PSM1/local/measured_cp" or not trial_running:
            continue

        msg = deserialize_message(data, msg_types[topic])
       
        t_sec = (timestamp - t0) * 1e-9

        time_stamps.append(t_sec)
        x_positions.append(msg.pose.position.x)
        y_positions.append(msg.pose.position.y)
        z_positions.append(msg.pose.position.z)
    # ---- end of rosbag iteration ----

    print(f"Evaluated datapoints during RUNNING state: {len(x_positions)}.")
    print(f"Number contacts during RUNNING state: {contact_count}")

    if len(x_positions) == 0:
        print("No data points found during RUNNING state.")
        return

    # ---- numpy conversions ----
    time_stamps = np.array(time_stamps)
    x_positions = np.array(x_positions)
    y_positions = np.array(y_positions)
    z_positions = np.array(z_positions)

    loop_time_stamps = np.array(loop_time_stamps)
    loop_centers = np.array(loop_centers)

    # ---- Distance computation ----
    distances = np.zeros(len(loop_centers))

    for i, (loopcenter, state) in enumerate(zip(loop_centers, trial_states)):

        if state not in state_to_segment:
            distances[i] = np.nan
            continue

        idx_a, idx_b = state_to_segment[state]
        a = given_wirepoints[idx_a]
        b = given_wirepoints[idx_b]

        distances[i] = point_to_segment_distance(loopcenter, a, b)

    # ---- Varianz berechnen ----
    #position_variance = np.var(position_errors)
    #print(f"Positionsvarianz relativ zum Wire: {position_variance:.8f} m²")

    force_time_stamps = np.array(force_time_stamps)
    fx_values = np.array(fx_values)
    fy_values = np.array(fy_values)
    fz_values = np.array(fz_values)

    # ---- Calculations ----
    #mean_x = np.mean(x_positions)
    #print(f"Mittelwert x-Position (RUNNING): {mean_x:.6f} m")

    force_values = np.sqrt(fx_values**2 + fy_values**2 + fz_values**2)

    print(f"Benötigte Zeit: {time_stamps[-1] - time_stamps[0]:.2f} s")

    # ---- helper function to add state change markers ----
    def add_state_markers():
        for t, label in zip(state_change_times, state_change_labels):
            plt.axvline(x=t, linestyle=":", linewidth=2, color="r")

    # ---- Plot Position ----
    plt.figure(figsize=(10, 4))
    plt.plot(time_stamps, x_positions, label="x")
    plt.plot(time_stamps, y_positions, label="y")
    plt.plot(time_stamps, z_positions, label="z")
    add_state_markers()
    plt.xlabel("time since start [s]")
    plt.ylabel("position [m]")
    plt.title("PSM1 position during RUNNING")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ---- Plot Forces ----
    if len(force_time_stamps) > 0:
        plt.figure(figsize=(10, 4))
        #plt.plot(force_time_stamps, fx_values, label="Fx")
        #plt.plot(force_time_stamps, fy_values, label="Fy")
        #plt.plot(force_time_stamps, fz_values, label="Fz")
        #plt.plot(force_time_stamps, force_values, label="|F|", linestyle="--", color="k")
        plt.plot(force_time_stamps, force_values, label="|F|")
        add_state_markers()
        plt.xlabel("time since start [s]")
        plt.ylabel("accumulated force magnitude [N]")
        plt.title("Haply Output Forces")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()
    else:
        print("No force data found.")

    # ---- Plot Position deviations ----
    plt.figure(figsize=(10,4))
    plt.plot(loop_time_stamps, distances)
    add_state_markers()
    plt.xlabel("time since start [s]")
    plt.ylabel("distance to wire [m]")
    plt.title("Position deviation from wire")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
