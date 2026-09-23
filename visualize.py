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

# ---------- DARK / GLOWY THEME COLORS ----------
BG_COLOR = "#0d0c0a"
PANE_COLOR = "#151210"
GRID_COLOR = "#3a2c22"
TEXT_COLOR = "#f2ece2"

COLOR_PICK = "#ff5a3d"      # glowing red-orange
COLOR_PLACE = "#39ff9d"     # glowing green
COLOR_HOME = "#9aa0a6"      # soft grey
COLOR_ARM = "#ffb347"       # glowing amber (matches site accent)
COLOR_GRIP_CLOSED = "#ff3d3d"
COLOR_GRIP_OPEN = "#e8e8e8"
COLOR_OBJECT = "#ff2ec4"    # glowing magenta


def glow_plot(ax, xs, ys, zs, color, base_lw=4, layers=4):
    """Draws the same line several times with growing width + shrinking
    alpha underneath the crisp top line, to fake a neon glow."""
    for i in range(layers, 0, -1):
        ax.plot(xs, ys, zs, "-", color=color,
                 linewidth=base_lw + i * 3, alpha=0.05, solid_capstyle="round")
    ax.plot(xs, ys, zs, "-o", color=color, linewidth=base_lw, markersize=6,
             markerfacecolor=BG_COLOR, markeredgecolor=color, markeredgewidth=1.5,
             solid_capstyle="round", zorder=4)


def glow_scatter(ax, pos, color, s=90, marker="o", layers=4, **kwargs):
    """Same trick for point markers: a soft halo behind a crisp marker."""
    for i in range(layers, 0, -1):
        ax.scatter(*pos, color=color, s=s + i * 220, marker=marker,
                    alpha=0.05, linewidths=0)
    return ax.scatter(*pos, color=color, s=s, marker=marker,
                       edgecolor=color, linewidths=0.8, **kwargs)


def style_dark_axes(ax):
    ax.set_facecolor(BG_COLOR)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor(PANE_COLOR)
        axis.pane.set_alpha(1.0)
        axis._axinfo["grid"]["color"] = GRID_COLOR
        axis._axinfo["grid"]["linewidth"] = 0.5
        axis.line.set_color(GRID_COLOR)
        axis.label.set_color(TEXT_COLOR)
    ax.tick_params(colors=TEXT_COLOR, labelsize=8)
    ax.title.set_color(TEXT_COLOR)


def build_animation(frames, out_path="pick_and_place.gif", fps=15):
    fig = plt.figure(figsize=(8, 7))
    fig.patch.set_facecolor(BG_COLOR)
    ax = fig.add_subplot(111, projection="3d")

    def set_view_limits():
        # Same axis ranges/scale as before — only the look changes.
        ax.set_xlim(-0.7, 0.7)
        ax.set_ylim(-0.7, 0.7)
        ax.set_zlim(0.0, 0.9)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        style_dark_axes(ax)

    def draw_frame(i):
        ax.cla()
        set_view_limits()
        f = frames[i]

        joint_positions, ee_pos = arm.forward_kinematics(f["angles"])

        # --- static scene markers (glowing) ---
        glow_scatter(ax, PICK_XYZ, COLOR_PICK, s=70, marker="o", label="Pick area")
        glow_scatter(ax, PLACE_XYZ, COLOR_PLACE, s=70, marker="s", label="Place area")
        glow_scatter(ax, HOME_XYZ, COLOR_HOME, s=40, marker="^", layers=2, label="Home")

        # --- robot arm links (glowing amber) ---
        xs, ys, zs = joint_positions[:, 0], joint_positions[:, 1], joint_positions[:, 2]
        glow_plot(ax, xs, ys, zs, COLOR_ARM, base_lw=4, layers=4)
        # attach the "Robot arm" legend entry without redrawing
        ax.plot([], [], "-o", color=COLOR_ARM, linewidth=4, markersize=6,
                 markerfacecolor=BG_COLOR, label="Robot arm")

        # --- gripper state indicator at end-effector ---
        gripper_color = COLOR_GRIP_CLOSED if f["gripper_closed"] else COLOR_GRIP_OPEN
        glow_scatter(ax, ee_pos, gripper_color, s=140, marker="D", layers=3,
                     zorder=5, label="Gripper (closed)" if f["gripper_closed"] else "Gripper (open)")

        # --- the object: at rest (pick/place location) or carried by the gripper ---
        obj_label = "Object (carried)" if f["object_attached"] else "Object"
        glow_scatter(ax, f["object_pos"], COLOR_OBJECT, s=200, marker="*", layers=4,
                     zorder=6, label=obj_label)

        ax.set_title(f"RAPID: Pick & Place Simulation\nStep: {f['step_label']}", fontsize=11)

        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        legend = ax.legend(by_label.values(), by_label.keys(), loc="upper left", fontsize=8,
                            facecolor=PANE_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)
        legend.get_frame().set_alpha(0.9)

    anim = FuncAnimation(fig, draw_frame, frames=len(frames), interval=1000 // fps)
    anim.save(out_path, writer=PillowWriter(fps=fps), savefig_kwargs={"facecolor": BG_COLOR})
    plt.close(fig)
    print(f"Saved animation -> {out_path}  ({len(frames)} frames)")


if __name__ == "__main__":
    sim = PickAndPlaceSimulation()
    frames = sim.run()
    sim.print_log()
    build_animation(frames, out_path="pick_and_place.gif")
