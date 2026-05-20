#!/usr/bin/env python3
from geometry_msgs.msg import Vector3


def haply_vector_to_psm(vector):
    """Convert a Haply vector message into the internal PSM/world frame."""
    return [
        float(vector.x),
        float(vector.y),
        -float(vector.z),
    ]


def psm_vector_to_haply_msg(vector):
    """Convert an internal PSM/world vector into a Haply Vector3 command."""
    return Vector3(
        x=float(vector[0]),
        y=float(vector[1]),
        z=-float(vector[2]),
    )


def haply_translation_delta_to_psm(dx, dy, dz):
    """Convert Haply translation deltas into the PSM/world translation frame."""
    return -dx, dy, -dz


def haply_rotation_delta_to_psm(droll, dpitch, dyaw):
    """Convert Haply rotation deltas into PSM joint-space roll/pitch/yaw deltas."""
    return dpitch, -dyaw, droll
