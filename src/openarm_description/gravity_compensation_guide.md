# OpenArm 重力补偿集成指南

## 做了什么

在 `openarm_hardware` 包的 ros2_control 硬件接口中，为电机 MIT 控制指令叠加了重力前馈补偿力矩，使机械臂不再依赖位置环 PID 硬撑抵抗重力。

## 修改了哪些文件

| 文件 | 改了什么 |
|------|---------|
| `openarm_hardware/src/dynamics.cpp` | **新增** — Dynamics 类，封装 KDL 动力学库，根据 URDF 和关节角度计算重力力矩 G(q) |
| `openarm_hardware/include/openarm_hardware/dynamics.hpp` | **新增** — Dynamics 类头文件 |
| `openarm_hardware/include/openarm_hardware/v10_simple_hardware.hpp` | 添加重力补偿相关成员变量（开关、缩放系数、URDF 路径、动力学实例等） |
| `openarm_hardware/src/v10_simple_hardware.cpp` | `parse_config()` 新增参数读取；`on_init()` 中初始化 Dynamics；`write()` 中叠加 `gravity_scale * G(q)` 到前馈力矩 |
| `openarm_hardware/CMakeLists.txt` | 添加 KDL、URDF、Eigen 依赖，加入 `dynamics.cpp` 编译 |
| `openarm_hardware/package.xml` | 添加 `kdl_parser`、`orocos_kdl`、`urdf` 依赖 |
| `openarm_description/.../openarm.bimanual.ros2_control.xacro` | 左右臂各添加 5 个重力补偿参数 |
| `openarm_description/.../openarm.ros2_control.xacro` | 单臂添加 5 个重力补偿参数 |

## 怎么用

### 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/openarm/ros2_ws && colcon build --packages-select openarm_hardware
source install/setup.bash
```

### 启动前生成 URDF（每次重启后需要重新做）

```bash
ros2 run xacro xacro \
  ~/openarm/ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro \
  bimanual:=true ros2_control:=true hand:=true \
  > /tmp/openarm_bimanual.urdf
```

### 启动

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py
```

### 调节参数

在 xacro 中修改后，重新生成 URDF 并重启 launch 即可，**不需要重新编译**：

- `use_gravity_compensation` — `true`/`false` 开关
- `gravity_scale` — 补偿强度，1.0 为全额补偿，建议从 1.0 起微调（步长 0.05）
- `gravity_tip_link` — 设为 `openarm_right_hand` 包含夹爪质量，设为 `openarm_right_link7` 则不包含

### 验证效果

执行同一 trajectory 对比最终关节位置与目标值的偏差。启用重力补偿后，重力负载关节（joint1、joint4）的位置跟踪精度应显著提升。
