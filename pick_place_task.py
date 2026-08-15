"""
pick_place_task.py
-------------------
Defines the fixed joint-angle waypoints (Home / Pick / Place, each with
an "approach" pose above it) and runs the 8-step pick-and-place state
machine described in the project brief:

    1. Start at Home
    2. Move to Pick position
    3. Lower the end-effector
    4. Activate simulated gripper
    5. Lift the object
    6. Move to Place position
    7. Release the object
    8. Return to Home

Every waypoint below is just a hand-picked list of 6 joint angles
(degrees) -- no inverse kinematics, no path planning. Motion between
waypoints is plain linear interpolation in joint space.
"""

import numpy as np
from robot_arm import RobotArm, JOINT_NAMES

# ---------------------------------------------------------------------------
# Hand-picked joint-angle waypoints (degrees): [base, shoulder, elbow,
# wrist_roll, wrist_pitch, wrist_yaw]
# ---------------------------------------------------------------------------
WAYPOINTS = {
    "HOME":           [0,   -20,  30,  0,  20,  0],
    "PICK_APPROACH":  [50,  -10,  40,  0,  40,  0],
    "PICK":           [50,   10,  55,  0,  55,  0],   # lowered onto object
    "PLACE_APPROACH": [-50, -10,  40,  0,  40,  0],
    "PLACE":          [-50,  10,  55,  0,  55,  0],   # lowered to place
}

# The 10 discrete steps of the task, in order.
# Each step is (step_name, target_waypoint, gripper_action, n_frames)
#   gripper_action: None | "CLOSE" | "OPEN"
STEP_SEQUENCE = [
    ("Start at HOME",                 "HOME",           None,    1),
    ("Move to Pick approach",         "PICK_APPROACH",  None,    25),
    ("Lower end-effector to object",  "PICK",           None,    15),
    ("Activate gripper (grasp)",      "PICK",           "CLOSE", 8),
    ("Lift object",                   "PICK_APPROACH",  None,    15),
    ("Move to Place approach",        "PLACE_APPROACH", None,    25),
    ("Lower to place position",       "PLACE",          None,    15),
    ("Release gripper (place)",       "PLACE",          "OPEN",  8),
    ("Lift clear of place",           "PLACE_APPROACH", None,    15),
    ("Return to HOME",                "HOME",            None,   25),
]


def interpolate_joint_space(start_deg, end_deg, n_frames):
    """Linear interpolation between two joint-angle sets, in degrees."""
    start = np.array(start_deg, dtype=float)
    end = np.array(end_deg, dtype=float)
    if n_frames <= 1:
        return [end.tolist()]
    return [ (start + (end - start) * t).tolist()
             for t in np.linspace(0, 1, n_frames) ]


class PickAndPlaceSimulation:
    """
    Drives the robot through the full task and records every frame:
    joint angles, gripper state, end-effector position, and whether
    the object is currently attached to the gripper.
    """

    def __init__(self):
        self.arm = RobotArm()
        self.frames = []      # list of dicts, one per animation frame
        self.log = []         # human-readable step log

    def run(self):
        current_angles = WAYPOINTS["HOME"]
        gripper_closed = False
        object_attached = False
        # Where the object rests when NOT attached to the gripper.
        # Starts at the pick location; becomes the place location after release.
        _, object_rest_pos = self.arm.forward_kinematics(WAYPOINTS["PICK"])
        _, place_rest_pos = self.arm.forward_kinematics(WAYPOINTS["PLACE"])

        for step_name, target_key, gripper_action, n_frames in STEP_SEQUENCE:
            target_angles = WAYPOINTS[target_key]
            path = interpolate_joint_space(current_angles, target_angles, n_frames)

            for angles in path:
                _, ee_pos = self.arm.forward_kinematics(angles)
                object_pos = ee_pos if object_attached else object_rest_pos
                self.frames.append({
                    "angles": angles,
                    "gripper_closed": gripper_closed,
                    "object_attached": object_attached,
                    "ee_pos": ee_pos,
                    "object_pos": object_pos,
                    "step_label": step_name,
                })

            # Apply the gripper action AFTER reaching the target pose
            if gripper_action == "CLOSE":
                gripper_closed = True
                object_attached = True
            elif gripper_action == "OPEN":
                gripper_closed = False
                object_attached = False
                object_rest_pos = place_rest_pos  # object now sits at the place location
                # log the extra "released" frame with updated gripper state
                _, ee_pos = self.arm.forward_kinematics(target_angles)
                self.frames.append({
                    "angles": target_angles,
                    "gripper_closed": gripper_closed,
                    "object_attached": object_attached,
                    "ee_pos": ee_pos,
                    "object_pos": object_rest_pos,
                    "step_label": step_name,
                })

            current_angles = target_angles
            self.log.append(
                f"[STEP] {step_name:32s} -> joints(deg)={[round(a,1) for a in target_angles]} "
                f"gripper={'CLOSED' if gripper_closed else 'OPEN'}"
            )

        return self.frames

    def print_log(self):
        print("=" * 78)
        print("PICK-AND-PLACE TASK LOG")
        print("=" * 78)
        for line in self.log:
            print(line)
        print("=" * 78)
        print(f"Total animation frames generated: {len(self.frames)}")


if __name__ == "__main__":
    sim = PickAndPlaceSimulation()
    sim.run()
    sim.print_log()
