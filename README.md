# OpenArm Sim2Real

RL policy training and deployment for [OpenArm](https://github.com/openarm/openarm_ros2) V10 right arm, powered by [Isaac Lab](https://isaac-sim.github.io/IsaacLab/)


---

https://github.com/user-attachments/assets/59a45b66-afeb-4696-916c-e93c05e092f5

## Installation

### Isaac Lab (training only)

Follow the [Isaac Lab installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html).

### OpenArm Sim2Real
Clone this repo **outside** the IsaacLab directory.
```bash
git clone https://github.com/CROBOT974/openarm_sim2real.git
cd openarm_sim2real
pip install -e source/openarm
```

### ROS 2 deployment prerequisites

Requires ROS 2 Humble + PyTorch + Openarm ROS2 package.

The openarm ROS2 prerequisites installation guide is available [here](https://docs.openarm.dev/1.0/software/ros2/control).

**IMPORTANT: Gravity compensation**

Real robot deployment **must** enable gravity compensation in `openarm_hardware`,
otherwise the RL policy cannot track joint1/joint4 accurately.
This blog provides an approriate workflow for [gravity compensation](https://blog.csdn.net/qq_53520547/article/details/160255892).

---

## Training (Isaac Lab)

Requires Isaac Lab conda environment with `isaaclab_tasks`, `isaaclab_rl`, `rsl-rl-lib`.

### Install extension

```bash
cd openarm_sim2real
pip install -e source/openarm
```

### Train

```bash
python scripts/rsl_rl/train.py --task OpenArm-Right-OpenArmRightReachEnvCfg-v0 \
    --num_envs 4096 --max_iterations 1000 --headless

# With video
python scripts/rsl_rl/train.py --task OpenArm-Right-OpenArmRightReachEnvCfg-v0 \
    --num_envs 4096 --max_iterations 1000 --video

# Resume from checkpoint
python scripts/rsl_rl/train.py --task OpenArm-Right-OpenArmRightReachEnvCfg-v0 \
    --num_envs 4096 --resume --load_run <run_folder_name>
```

Logs are saved to `logs/rsl_rl/openarm_ri_reach/<timestamp>/`.

### Playback and export

```bash
python scripts/rsl_rl/play.py --task OpenArm-Right-OpenArmRightReachEnvCfg-Play-v0 \
    --num_envs 1

# With specific checkpoint
python scripts/rsl_rl/play.py --task OpenArm-Right-OpenArmRightReachEnvCfg-Play-v0 \
    --num_envs 1 --load_run <run_folder_name>
```

The exported `policy.pt` goes to `logs/rsl_rl/openarm_ri_reach/<run>/exported/`.

### CLI arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--task` | `OpenArm-Right-OpenArmRightReachEnvCfg-v0` | Training task |
| `--num_envs` | 4096 | Parallel environments |
| `--max_iterations` | 550 | Training iterations |
| `--seed` | random | Random seed (-1 = random) |
| `--headless` | false | Disable GUI |
| `--video` | false | Record training video |
| `--resume` | false | Resume from checkpoint |
| `--load_run` | - | Run folder to resume from |

---

## Deployment (ROS 2)

### Build

The nested `openarm_sim2real/openarm_sim2real/` is the actual ROS 2 package.
Copy it to your `ros2_ws/src/`, then build:

```bash
cp -r openarm_sim2real/openarm_sim2real ~/openarm/ros2_ws/src/
cd ~/openarm/ros2_ws
colcon build --packages-select openarm_sim2real
source install/setup.bash
```

> **Note**: The repo root `openarm_sim2real/` contains training code and logs.
> Only the inner `openarm_sim2real/openarm_sim2real/` directory is a ROS 2 package.

### Fake hardware test

```bash
ros2 launch openarm_sim2real sim2real_forward_cmd.launch.py \
  model_path:=/path/to/policy.pt
```

### Real robot

```bash
# Terminal 1: bring up hardware
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=false arm_type:=v10

# Terminal 2: RL policy inference
ros2 launch openarm_sim2real sim2real_forward_cmd.launch.py \
  model_path:=/path/to/policy.pt
```

### Multi-target cycling

Modify `TARGET_CYCLE` in [rl_policy_node_cycle.py](openarm_sim2real/scripts/rl_policy_node_cycle.py),
then launch the cycle node (also accepts `model_path` parameter).

### Manual control

```bash
python3 send_forward_cmd.py 0.0 0.1 0.0 0.5 0.0 0.0 0.0
```

---

## Parameters

In [rl_policy_node.py](openarm_sim2real/scripts/rl_policy_node.py):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_DELTA_PER_STEP` | 0.06 rad | Max joint delta per inference step |
| `DECIMATION` | 2 | Inference every N frames (2 = 30 Hz) |
| `EE_DIST_THRESHOLD` | 0.02 m | Stop when EE is within this distance of target |
| `ACTION_SCALE` | 0.5 | Scales RL action to joint position delta |
| `INFERENCE_RATE` | 60.0 Hz | Timer callback rate |

---

## Dependencies
**Gravity Compensation:**
- [gravity compensation](https://blog.csdn.net/qq_53520547/article/details/160255892)

**Deployment:**
- [openarm_ros2](https://github.com/enactic/openarm_ros2)
- `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros` (ROS 2 Humble)
- `torch`, `numpy`

**Training (additional):**
- [Isaac Lab](https://isaac-sim.github.io/IsaacLab/)
- `rsl-rl-lib` >= 3.0.1

---

## Citation

```bibtex

@software{Openarm-Sim2Real,
  author = {Chi, Cheng and LiKang, Song and Jiaxi Zheng},
  title = {OpenArm Sim2Real: RL Training and Deployment for OpenArm},
  url = {https://github.com/CROBOT974/openarm_sim2real},
  version = {1.0.0},
  year = {2026}
}
```
