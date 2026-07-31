# 🤖 OpenArm Sim2Real Toolkit

<p align="center">
  <a href="https://docs.ros.org/en/humble/"><img src="https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat&logo=ros" alt="ROS2"></a>
  <a href="https://isaac-sim.github.io/IsaacLab/main/index.html"><img src="https://img.shields.io/badge/Sim-Isaac%20Lab-4B9CD3?style=flat&logo=nvidia" alt="Isaac Lab"></a>
  <a href="https://github.com/leggedrobotics/rsl_rl"><img src="https://img.shields.io/badge/RL-RSL--RL-00A651?style=flat&logo=pytorch" alt="RSL-RL"></a>
  <a href="https://www.apache.org/licenses/LICENSE-2.0"><img src="https://img.shields.io/badge/License-Apache%202.0-blue?style=flat" alt="License"></a>
</p>

<p align="center">
  <b>RL policy training and deployment for OpenArm V10 right arm</b><br>
  Sim-to-real via <a href="https://isaac-sim.github.io/IsaacLab/">Isaac Lab</a> &nbsp;|&nbsp;
  ROS 2 + ForwardCommandController &nbsp;|&nbsp;
  <a href="https://github.com/openarm/openarm_ros2">OpenArm</a>
</p>

---

https://github.com/user-attachments/assets/59a45b66-afeb-4696-916c-e93c05e092f5

---

## 📦 Installation

### 🧪 Isaac Lab (training only)

Refer to the [Isaac Lab installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html).

<details open>
<summary><b>OpenArm Sim2Real extension</b></summary>

> **Clone this repo outside the IsaacLab directory.**

```bash
git clone https://github.com/CROBOT974/openarm_sim2real.git
cd openarm_sim2real
pip install -e source/openarm
```

</details>

### 🤖 ROS 2 deployment prerequisites

| Requirement | Details |
|-------------|---------|
| ROS 2 | Humble |
| Python | `torch`, `numpy` |
| Robot SDK | [openarm_ros2](https://docs.openarm.dev/1.0/software/ros2/control) |

> **IMPORTANT — Gravity compensation:** Real robot deployment **must** enable gravity compensation in `openarm_hardware`, otherwise the RL policy cannot accurately track joints 1 & 4. See [this guide](https://blog.csdn.net/qq_53520547/article/details/160255892).

---

## 🚀 Training (Isaac Lab)

Requires Isaac Lab conda environment with `isaaclab_tasks`, `isaaclab_rl`, `rsl-rl-lib`.

<details>
<summary><b>▶ Click to expand — commands</b></summary>

```bash
cd openarm_sim2real
pip install -e source/openarm

# Train (headless)
python scripts/rsl_rl/train.py --task OpenArm-Right-OpenArmRightReachEnvCfg-v0 \
    --num_envs 4096 --max_iterations 1000 --headless

# Train (with video)
python scripts/rsl_rl/train.py --task OpenArm-Right-OpenArmRightReachEnvCfg-v0 \
    --num_envs 4096 --max_iterations 1000 --video

# Resume
python scripts/rsl_rl/train.py --task OpenArm-Right-OpenArmRightReachEnvCfg-v0 \
    --num_envs 4096 --resume --load_run <run_folder_name>
```

</details>

Logs → `logs/rsl_rl/openarm_ri_reach/<timestamp>/`

### ▶️ Playback & export

```bash
python scripts/rsl_rl/play.py --task OpenArm-Right-OpenArmRightReachEnvCfg-Play-v0 \
    --num_envs 1

# Specific checkpoint
python scripts/rsl_rl/play.py --task OpenArm-Right-OpenArmRightReachEnvCfg-Play-v0 \
    --num_envs 1 --load_run <run_folder_name>
```

The exported `policy.pt` is saved to `logs/rsl_rl/openarm_ri_reach/<run>/exported/`.

### ⚙️ CLI arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--task` | `OpenArm-Right-OpenArmRightReachEnvCfg-v0` | Training task |
| `--num_envs` | `4096` | Parallel environments |
| `--max_iterations` | `550` | Training iterations |
| `--seed` | random | `-1` for random |
| `--headless` | `false` | Disable GUI |
| `--video` | `false` | Record training video |
| `--resume` | `false` | Resume from checkpoint |
| `--load_run` | — | Run folder to resume from |

---

## 🤖 Deployment (ROS 2)

### 🔨 Build

The nested `openarm_sim2real/openarm_sim2real/` is the actual ROS 2 package.
Copy it to your `ros2_ws/src/`, then build:

```bash
cp -r openarm_sim2real/openarm_sim2real ~/openarm/ros2_ws/src/
cd ~/openarm/ros2_ws
colcon build --packages-select openarm_sim2real
source install/setup.bash
```

> **Note:** The repo root contains training code + logs. Only the inner `openarm_sim2real/` directory is a ROS 2 package.

### 🧪 Fake hardware test

```bash
ros2 launch openarm_sim2real sim2real_forward_cmd.launch.py \
  model_path:=/path/to/policy.pt
```

### 🦾 Real robot

```bash
# Terminal 1: bring up hardware
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=false arm_type:=v10

# Terminal 2: RL policy inference
ros2 launch openarm_sim2real sim2real_forward_cmd.launch.py \
  model_path:=/path/to/policy.pt
```

### 🔁 Multi-target cycling

Edit `TARGET_CYCLE` in [rl_policy_node_cycle.py](openarm_sim2real/scripts/rl_policy_node_cycle.py), then launch the cycle node.

### 🎮 Manual control

```bash
python3 send_forward_cmd.py 0.0 0.1 0.0 0.5 0.0 0.0 0.0
```

---

## 🎛️ Parameters

In [rl_policy_node.py](openarm_sim2real/scripts/rl_policy_node.py):

| Parameter | Default | Description |
|-----------|:------:|-------------|
| `MAX_DELTA_PER_STEP` | `0.06` rad | Max joint delta per inference step |
| `DECIMATION` | `2` | Inference every N frames (→ 30 Hz) |
| `EE_DIST_THRESHOLD` | `0.02` m | Stop when EE within this distance |
| `ACTION_SCALE` | `0.5` | RL action → joint position scale |
| `INFERENCE_RATE` | `60.0` Hz | Timer callback rate |

---

## 📚 Dependencies

| Category | Packages |
|----------|----------|
| **Deployment** | `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros` (ROS 2 Humble), `torch`, `numpy` |
| **Gravity Compensation** | [Guide & patches](https://blog.csdn.net/qq_53520547/article/details/160255892) |
| **Training** | [Isaac Lab](https://isaac-sim.github.io/IsaacLab/), `rsl-rl-lib` ≥ 3.0.1 |
| **Robot SDK** | [openarm_ros2](https://github.com/enactic/openarm_ros2) |

---

## 📖 Citation

```bibtex
@software{Openarm-Sim2Real,
  author = {Chi Cheng, LiKang Song, Jiaxi Zheng and Dixia Fan},
  title = {OpenArm Sim2Real: RL Training and Deployment for OpenArm},
  url = {https://github.com/CROBOT974/openarm_sim2real},
  version = {1.0.0},
  year = {2026}
}
```

<p align="center">
  <sub>Built with ❤️ on <a href="https://github.com/isaac-sim/IsaacLab">Isaac Lab</a> and <a href="https://github.com/enactic/openarm_ros2">OpenArm</a></sub>
</p>
