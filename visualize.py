"""
visualize.py
------------
Renders the pick-and-place run as a 3D animated GIF: the robot arm,
the pick location, the place location, and the object being carried
once the gripper closes on it.

Run:  python3 visualize.py
Output: pick_and_place.gif  (in the same folder)
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from robot_arm import RobotArm
from pick_place_task import PickAndPlaceSimulation, WAYPOINTS

arm = RobotArm()

# Fixed world positions for drawing markers: computed once from the
# PICK / PLACE waypoints via forward kinematics (visualization only).
_, PICK_XYZ = arm.forward_kinematics(WAYPOINTS["PICK"])
_, PLACE_XYZ = arm.forward_kinematics(WAYPOINTS["PLACE"])
_, HOME_XYZ = arm.forward_kinematics(WAYPOINTS["HOME"])


def build_animation(frames, out_path="pick_and_place.gif", fps=15):
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    def set_view_limits():
        ax.set_xlim(-0.7, 0.7)
        ax.set_ylim(-0.7, 0.7)
        ax.set_zlim(0.0, 0.9)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

    def draw_frame(i):
        ax.cla()
        set_view_limits()
        f = frames[i]

        joint_positions, ee_pos = arm.forward_kinematics(f["angles"])

        # --- static scene markers ---
        ax.scatter(*PICK_XYZ, color="tab:red", s=70, marker="o", label="Pick area")
        ax.scatter(*PLACE_XYZ, color="tab:green", s=70, marker="s", label="Place area")
        ax.scatter(*HOME_XYZ, color="gray", s=40, marker="^", label="Home")

        # --- robot arm links ---
        xs, ys, zs = joint_positions[:, 0], joint_positions[:, 1], joint_positions[:, 2]
        ax.plot(xs, ys, zs, "-o", color="tab:blue", linewidth=4, markersize=6,
                markerfacecolor="black", label="Robot arm")

        # --- gripper state indicator at end-effector ---
        gripper_color = "tab:orange" if f["gripper_closed"] else "white"
        ax.scatter(*ee_pos, color=gripper_color, edgecolor="black", s=140,
                   marker="D", zorder=5, label="Gripper (closed)" if f["gripper_closed"] else "Gripper (open)")

        # --- the object: at rest (pick/place location) or carried by the gripper ---
        obj_label = "Object (carried)" if f["object_attached"] else "Object"
        ax.scatter(*f["object_pos"], color="purple", s=200, marker="*", zorder=6, label=obj_label)

        ax.set_title(f"Pick & Place Simulation\nStep: {f['step_label']}", fontsize=11)

        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc="upper left", fontsize=8)

    anim = FuncAnimation(fig, draw_frame, frames=len(frames), interval=1000 // fps)
    anim.save(out_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    print(f"Saved animation -> {out_path}  ({len(frames)} frames)")


if __name__ == "__main__":
    sim = PickAndPlaceSimulation()
    frames = sim.run()
    sim.print_log()
    build_animation(frames, out_path="pick_and_place.gif")
