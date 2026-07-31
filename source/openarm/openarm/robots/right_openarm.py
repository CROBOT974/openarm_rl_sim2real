"""Configuration of OpenArm right-arm-only robot (bimanual base, right arm).

The following configurations are available:

* :obj:`OPENARM_RIGHT_CFG`: OpenArm with right arm only.
* :obj:`OPENARM_RIGHT_HIGH_PD_CFG`: OpenArm with right arm only and stiffer PD control.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuration
##

_OPENARM_USD_DIR = os.path.join(os.path.dirname(__file__), "usd")

OPENARM_RIGHT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.path.join(_OPENARM_USD_DIR, "openarm_bimanual.usd"),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "openarm_right_joint.*": 0.0,
            "openarm_right_finger_joint.*": 0.0,
        },
    ),
    actuators={
        "openarm_arm": ImplicitActuatorCfg(
            joint_names_expr=["openarm_right_joint[1-7]"],
            velocity_limit_sim={
                "openarm_right_joint[1-2]": 2.175,
                "openarm_right_joint[3-4]": 2.175,
                "openarm_right_joint[5-7]": 2.61,
            },
            effort_limit_sim={
                "openarm_right_joint[1-2]": 40.0,
                "openarm_right_joint[3-4]": 27.0,
                "openarm_right_joint[5-7]": 7.0,
            },
            # Per-joint PD gains matching real OpenArm hardware (control_gains.yaml)
            stiffness={
                "openarm_right_joint[1-3]": 70.0,
                "openarm_right_joint4": 60.0,
                "openarm_right_joint[5-7]": 10.0,
            },
            damping={
                "openarm_right_joint1": 2.75,
                "openarm_right_joint2": 2.5,
                "openarm_right_joint3": 2.0,
                "openarm_right_joint4": 2.0,
                "openarm_right_joint5": 0.7,
                "openarm_right_joint6": 0.6,
                "openarm_right_joint7": 0.5,
            },
            armature={
                "openarm_right_joint1": 0.0015,
                "openarm_right_joint2": 0.0015,
                "openarm_right_joint3": 0.032,
                "openarm_right_joint4": 0.032,
                "openarm_right_joint5": 0.0018,
                "openarm_right_joint6": 0.0018,
                "openarm_right_joint7": 0.0018,
            },
            friction={
                "openarm_right_joint[1-3]": 0.2,
                "openarm_right_joint[4-7]": 0.03,
            },
            viscous_friction={
                "openarm_right_joint[1-3]": 0.1,
                "openarm_right_joint[4-7]": 0.02,
            },
        ),
        "openarm_gripper": ImplicitActuatorCfg(
            joint_names_expr=["openarm_right_finger_joint.*"],
            velocity_limit_sim=0.2,
            effort_limit_sim=333.33,
            stiffness=5,
            damping=0.1,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of OpenArm right-arm-only robot (bimanual base, right arm only)."""

OPENARM_RIGHT_HIGH_PD_CFG = OPENARM_RIGHT_CFG.copy()
OPENARM_RIGHT_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
OPENARM_RIGHT_HIGH_PD_CFG.actuators["openarm_arm"].stiffness = {
    "openarm_right_joint[1-3]": 350.0,
    "openarm_right_joint4": 300.0,
    "openarm_right_joint[5-7]": 50.0,
}
OPENARM_RIGHT_HIGH_PD_CFG.actuators["openarm_arm"].damping = {
    "openarm_right_joint1": 55.0,
    "openarm_right_joint2": 50.0,
    "openarm_right_joint3": 40.0,
    "openarm_right_joint4": 40.0,
    "openarm_right_joint5": 14.0,
    "openarm_right_joint6": 12.0,
    "openarm_right_joint7": 10.0,
}
OPENARM_RIGHT_HIGH_PD_CFG.actuators["openarm_gripper"].stiffness = 2e3
OPENARM_RIGHT_HIGH_PD_CFG.actuators["openarm_gripper"].damping = 1e2
"""Configuration of OpenArm right-arm-only robot with stiffer PD control.

This configuration is useful for task-space control using differential IK.
"""
