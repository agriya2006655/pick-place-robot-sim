"""
cycle_time.py
--------------
Extension to the pick-and-place simulation: estimates how long one
full pick-and-place CYCLE takes (Home -> Pick -> Place -> Home), and
compares that baseline against two "what if we moved faster" scenarios
to see how much the cycle time could realistically be reduced.

HOW CYCLE TIME IS CALCULATED (industry-standard simplified method)
--------------------------------------------------------------------
For a move from one joint-angle waypoint to another, industrial robots
move all 6 joints AT THE SAME TIME (not one after another). So the time
for that move is decided by whichever single joint has to travel
furthest relative to its own speed limit -- the "slowest" joint for
that particular move. That's:

    segment_time = max( |angle_change_of_joint_i| / max_speed_of_joint_i )
                   over all 6 joints i

This is the same logic real robot controllers and offline programming
tools (RobotStudio, RoboDK, etc.) use for a first-pass cycle time
estimate, before adding acceleration-ramp corrections.

Total cycle time = sum of every segment's time, plus a small fixed
"dwell" time each time the gripper opens or closes (it isn't
instant -- a pneumatic gripper takes a moment to actuate).

WHAT WE COMPARE
--------------------------------------------------------------------
  Baseline   : conservative joint speed limits + 0.4s gripper actuation
  Optimized 1: joint speed limits raised 20% (faster servo tuning)
  Optimized 2: Optimized 1 + a quicker 0.2s gripper (faster pneumatic
               valve) -- still physically reasonable, nothing unsafe.

Run: python3 cycle_time.py
Outputs: cycle_time_comparison.png, prints a summary table.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pick_place_task import WAYPOINTS, STEP_SEQUENCE

# ---------------------------------------------------------------------------
# Max joint speeds (degrees/second). These are realistic, conservative
# figures for a small 6-axis industrial arm (broadly in line with
# published specs for robots in the Fanuc LR Mate / M-20 class).
# ---------------------------------------------------------------------------
BASELINE_JOINT_SPEED = {
    "deg_per_sec": [150, 150, 180, 360, 360, 450],  # base..wrist_yaw
    "gripper_dwell_sec": 0.4,
    "label": "Baseline",
}

OPTIMIZED_1 = {
    "deg_per_sec": [s * 1.20 for s in BASELINE_JOINT_SPEED["deg_per_sec"]],  # +20% servo speed
    "gripper_dwell_sec": 0.4,
    "label": "Optimized 1 (+20% joint speed)",
}

OPTIMIZED_2 = {
    "deg_per_sec": [s * 1.20 for s in BASELINE_JOINT_SPEED["deg_per_sec"]],
    "gripper_dwell_sec": 0.2,  # faster pneumatic gripper actuation
    "label": "Optimized 2 (+20% joint speed, faster gripper)",
}

SCENARIOS = [BASELINE_JOINT_SPEED, OPTIMIZED_1, OPTIMIZED_2]


def segment_time(start_deg, end_deg, joint_speeds):
    """Time for one joint-space move = slowest joint's travel time."""
    start = np.array(start_deg, dtype=float)
    end = np.array(end_deg, dtype=float)
    speeds = np.array(joint_speeds, dtype=float)
    delta = np.abs(end - start)
    per_joint_time = delta / speeds
    return float(np.max(per_joint_time))


def compute_cycle_time(scenario):
    current_angles = WAYPOINTS["HOME"]
    total_time = 0.0
    breakdown = []

    for step_name, target_key, gripper_action, _n_frames in STEP_SEQUENCE:
        target_angles = WAYPOINTS[target_key]
        t = segment_time(current_angles, target_angles, scenario["deg_per_sec"])
        total_time += t
        breakdown.append((step_name, "move", round(t, 3)))

        if gripper_action in ("CLOSE", "OPEN"):
            total_time += scenario["gripper_dwell_sec"]
            breakdown.append((step_name, f"gripper {gripper_action.lower()}",
                               round(scenario["gripper_dwell_sec"], 3)))

        current_angles = target_angles

    return total_time, breakdown


def main():
    results = []
    for scenario in SCENARIOS:
        total, breakdown = compute_cycle_time(scenario)
        results.append((scenario["label"], total, breakdown))

    print("=" * 70)
    print("CYCLE TIME COMPARISON")
    print("=" * 70)
    baseline_time = results[0][1]
    for label, total, _ in results:
        pct = (1 - total / baseline_time) * 100
        print(f"{label:42s} {total:6.2f} sec   ({pct:+.1f}% vs baseline)")
    print("=" * 70)

    best_label, best_time, _ = min(results, key=lambda r: r[1])
    print(f"Fastest: {best_label} -> {best_time:.2f} sec\n")

    # Detailed breakdown for the baseline, for the report
    print("Baseline step-by-step breakdown:")
    for step_name, kind, t in results[0][2]:
        print(f"  {step_name:32s} [{kind:14s}] {t:.3f} sec")

    # --- chart ---
    labels = [r[0] for r in results]
    times = [r[1] for r in results]
    colors = ["#4C72B0", "#DD8452", "#55A868"]

    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(labels, times, color=colors)
    ax.set_ylabel("Cycle time (seconds)")
    ax.set_title("Pick-and-Place Cycle Time: Baseline vs Optimized Scenarios")
    for bar, t in zip(bars, times):
        ax.text(bar.get_x() + bar.get_width() / 2, t + 0.05, f"{t:.2f}s",
                 ha="center", va="bottom", fontsize=10)
    plt.xticks(rotation=10, ha="right")
    plt.tight_layout()
    plt.savefig("cycle_time_comparison.png", dpi=150)
    print("\nSaved cycle_time_comparison.png")


if __name__ == "__main__":
    main()
