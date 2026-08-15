"""
robot_arm.py
------------
A minimal 6-axis (6R) robot arm model.

We only need FORWARD kinematics here: given 6 joint angles, compute
where every joint and the end-effector end up in 3D space so we can
draw the arm and know where the gripper (and any object it's holding)
currently is.

No inverse kinematics is used anywhere in this project. Every pose the
robot visits (Home, Pick, Place, ...) is simply a hand-picked set of
6 joint angles -- exactly like teaching a real robot by jogging it to
a position and hitting "record".
"""

import numpy as np

# ---------------------------------------------------------------------------
# DH (Denavit-Hartenberg) parameters for a generic elbow-type 6R arm.
# Each row is (a, alpha, d) for joint i. theta_i is the variable joint angle.
#   a     : link length along x_i
#   alpha : link twist about x_i (radians)
#   d     : link offset along z_(i-1)
# Units: meters / radians
# ---------------------------------------------------------------------------
DH_A     = [0.00, 0.30, 0.25, 0.00, 0.00, 0.00]
DH_ALPHA = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]
DH_D     = [0.40, 0.00, 0.00, 0.20, 0.00, 0.10]

NUM_JOINTS = 6


def dh_transform(theta, a, alpha, d):
    """Standard DH homogeneous transform for one joint."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,        sa,       ca,      d],
        [0,         0,        0,      1],
    ])


class RobotArm:
    """A 6-axis serial manipulator, forward-kinematics only."""

    def __init__(self):
        self.joint_angles = np.zeros(NUM_JOINTS)  # radians

    def set_joint_angles(self, angles_deg):
        self.joint_angles = np.radians(np.array(angles_deg, dtype=float))

    def forward_kinematics(self, angles_deg=None):
        """
        Returns:
            joint_positions: (7, 3) array -- base + each of the 6 joints,
                              i.e. the polyline to draw for the arm.
            end_effector_pos: (3,) position of the gripper tip (world frame).
        """
        angles = (np.radians(np.array(angles_deg, dtype=float))
                  if angles_deg is not None else self.joint_angles)

        T = np.eye(4)
        positions = [T[:3, 3].copy()]  # base at origin

        for i in range(NUM_JOINTS):
            Ti = dh_transform(angles[i], DH_A[i], DH_ALPHA[i], DH_D[i])
            T = T @ Ti
            positions.append(T[:3, 3].copy())

        return np.array(positions), positions[-1]


# Human-readable joint names, used only for logging.
JOINT_NAMES = [
    "Base (yaw)", "Shoulder", "Elbow", "Wrist roll", "Wrist pitch", "Wrist yaw"
]
