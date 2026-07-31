from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


# Custom minimal model config without deprecated fields
# (RslRlMLPModelCfg has deprecated fields that break rsl-rl >= 4.0)


@configclass
class MLPModelCfg:
    """Configuration for the MLP model (compatible with rsl-rl >= 4.0)."""

    class_name: str = "MLPModel"
    """The model class name."""

    hidden_dims: list[int] = ...
    """The hidden dimensions of the MLP network."""

    activation: str = ...
    """The activation function for the MLP network."""

    obs_normalization: bool = False
    """Whether to normalize the observation for the model."""

    distribution_cfg: object | None = None
    """The configuration for the output distribution."""

    @configclass
    class DistributionCfg:
        """Configuration for the output distribution."""

        class_name: str = ...

    @configclass
    class GaussianDistributionCfg(DistributionCfg):
        """Configuration for the Gaussian output distribution."""

        class_name: str = "GaussianDistribution"
        init_std: float = ...
        std_type: str = "scalar"


@configclass
class OpenArmRightReachPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 550
    save_interval = 1000
    experiment_name = "openarm_ri_reach"
    run_name = ""
    resume = False
    empirical_normalization = False
    obs_groups = {
        "actor": ["policy"],
        "critic": ["critic"],
    }
    actor = MLPModelCfg(
        hidden_dims=[64, 64],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=MLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )
    critic = MLPModelCfg(
        hidden_dims=[64, 64],
        activation="elu",
        obs_normalization=False,
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.001,
        num_learning_epochs=8,
        num_mini_batches=4,
        learning_rate=1.0e-2,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
