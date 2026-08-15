# Automated Pick-and-Place Robot Simulation

A minimal 6-axis robot arm simulation: pick an object from Position A,
place it at Position B, return home. No computer vision, no AI, no ROS,
no inverse kinematics — every pose is a hand-picked set of joint angles,
and motion between poses is straight-line interpolation in joint space.

## Files

| File                  | Purpose                                                        |
|------------------------|-----------------------------------------------------------------|
| `robot_arm.py`         | 6-DOF arm model. Forward kinematics only (DH parameters).       |
| `pick_place_task.py`   | Waypoints (Home/Pick/Place) + the 10-step task state machine.   |
| `visualize.py`         | Renders the run as an animated 3D GIF (`pick_and_place.gif`).   |

## How it works

1. **`robot_arm.py`** — defines the 6 joints via standard DH parameters
   and computes forward kinematics: given 6 joint angles, where is every
   joint (and the gripper tip) in 3D space.

2. **`pick_place_task.py`** — defines 5 fixed waypoints as joint-angle
   sets (`HOME`, `PICK_APPROACH`, `PICK`, `PLACE_APPROACH`, `PLACE`) and
   runs the task as a sequence of 10 steps:

   ```
   Start at HOME
     -> Move to Pick approach
     -> Lower end-effector to object
     -> Activate gripper (grasp)
     -> Lift object
     -> Move to Place approach
     -> Lower to place position
     -> Release gripper (place)
     -> Lift clear of place
     -> Return to HOME
   ```

   Between waypoints, joint angles are linearly interpolated to produce
   smooth intermediate frames (no path planning needed since it's just
   two fixed endpoints).

3. **`visualize.py`** — steps through every frame, computes the arm's
   3D pose, and draws it: blue polyline = the arm's links, diamond =
   gripper (orange when closed), star = the object (follows the gripper
   once grasped). Saves the whole run as `pick_and_place.gif`.

## Run it

```bash
pip install numpy matplotlib pillow

python3 pick_place_task.py   # prints the step-by-step task log only
python3 visualize.py         # runs the task AND saves pick_and_place.gif
```

## Extending later

- Swap the hand-picked joint waypoints for real inverse kinematics once
  you want to specify Pick/Place in Cartesian (x, y, z) instead of joint
  angles.
- Replace the fixed `PICK` waypoint with a value read from a simulated
  or real sensor (this is where computer vision would plug in).
- Add joint velocity/acceleration limits for more realistic timing
  instead of plain linear interpolation.
