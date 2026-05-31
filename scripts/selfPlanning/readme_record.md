# Data Recording — Pick & Place

## Recording a new episode

### Update Changes
```bash
cd ~/ros2arm_ws
colcon build
source install/setup.bash
```

### Terminal 1 — camera + crosshair + RViz
```bash
cd ~/ros2arm_ws && source install/setup.bash
ros2 launch armpi_ultra_description teleop_rviz.launch.py
```

### Terminal 2 — teleop (must own the terminal for mouse/keyboard)
```bash
cd ~/ros2arm_ws && source install/setup.bash
python3 scripts/teleop_ik_v2.py
```

### Terminal 3 — bag recorder (auto-increments episode number)
```bash
cd ~/ros2arm_ws && source install/setup.bash
EPISODE=$(ls -d data/pick_place/episode_* 2>/dev/null | wc -l)
ros2 bag record \
  /aurora/rgb/crosshair \
  /teleop/ik_state \
  /teleop/recording \
  -o data/pick_place/episode_$EPISODE
```

**Workflow per episode:**
1. Start all three terminals
2. Arm moves to home1 automatically on teleop startup
3. Press `I` in the teleop terminal to mark recording start
4. Perform the pick-and-place demo
5. Press `O` to mark recording stop
6. `Ctrl+C` in Terminal 3 to save the bag

**Controls reminder:**
| Input | Action |
|---|---|
| Mouse left/right | Base rotation |
| Mouse up/down | Vertical position |
| Scroll | Reach in/out |
| Q / E | Pitch up/down |
| A / D | Wrist roll |
| Left click | Close gripper (step) |
| Right click | Open gripper (step) |
| H | Return to home1 |
| 1–9 | Sensitivity multiplier |

---

## Convert bags to HDF5

After recording all episodes:
```bash
cd ~/ros2arm_ws && source install/setup.bash
python3 scripts/bag_to_hdf5.py
```

Output:
```
data/hdf5/episode_0.hdf5
data/hdf5/episode_1.hdf5
...
data/hdf5/norm_bounds.npz
```

To convert a single episode:
```bash
python3 scripts/bag_to_hdf5.py --episode 3
```

---

## Verify a recording

Inspect bag info:
```bash
ros2 bag info data/pick_place/episode_0
```

Replay the bag (Terminal 1 must still be running):
```bash
ros2 bag play data/pick_place/episode_0
```
