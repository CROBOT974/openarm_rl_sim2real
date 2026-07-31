from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
from isaaclab_tasks.manager_based.manipulation.reach.config.openarm.right.reach_openarm_bi_env_cfg import ReachEnvCfg

from openarm.robots.right_openarm import OPENARM_RIGHT_CFG

##
# Environment configuration
##


@configclass
class OpenArmRightReachEnvCfg(ReachEnvCfg):
    """Configuration for the Right-Arm-Only OpenArm Reach Environment."""

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to OpenArm right arm only
        self.scene.robot = OPENARM_RIGHT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # override rewards
        self.rewards.right_end_effector_position_tracking.params["asset_cfg"].body_names = ["openarm_right_hand"]
        self.rewards.right_end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = [
            "openarm_right_hand"
        ]
        self.rewards.right_end_effector_orientation_tracking.params["asset_cfg"].body_names = ["openarm_right_hand"]

        # override actions
        self.actions.right_arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[
                "openarm_right_joint.*",
            ],
            scale=0.5,
            use_default_offset=True,
        )

        # override command generator body
        # end-effector is along z-direction
        self.commands.right_ee_pose.body_name = "openarm_right_hand"


@configclass
class OpenArmRightReachEnvCfg_PLAY(OpenArmRightReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
