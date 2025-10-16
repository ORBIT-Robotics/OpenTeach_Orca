"""
orca.py
=======
Robot wrapper for the ORCA robotic hand.

This class provides the standard robot API expected by OpenTeach:
    - get_state()
    - move_robot()
    - home_robot()

Instead of hard-coding joints, it *auto-loads* the correct joint names
and limits directly from the ORCA-core model YAML file, ensuring it is
always up-to-date with the hardware configuration.

The rest of OpenTeach (operators, teleop, etc.) can then stay generic.
"""

import os
from pathlib import Path
import yaml
import numpy as np

# Low-level ROS communication bridge
from openteach.ros_links.orca_control import DexArmControl


# ---------------------------------------------------------------------------
# Model selection
# ---------------------------------------------------------------------------
# Change this to 'orcahand_v1_left' or another version as needed
MODEL = "orcahand_v1_right"


def _load_model_yaml(model: str) -> dict:
    """
    Load the YAML configuration file for the selected ORCA hand model.
    """
    # Path to orca_core models directory
    orca_core_path = Path(__file__).parent.parent.parent / "orca_core" / "orca_core"
    cfg_path = orca_core_path / "models" / model / "config.yaml"
    
    if not cfg_path.exists():
        raise FileNotFoundError(
            f"ORCA model config not found at {cfg_path}. "
            f"Make sure orca_core is installed."
        )
    
    with open(cfg_path, 'r') as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# Main wrapper class
# ---------------------------------------------------------------------------

class OrcaHand:
    """
    OpenTeach robot wrapper for a single ORCA hand.
    Auto-syncs with ORCA-core's YAML and delegates ROS communication
    to the DexArmControl class.
    """

    def __init__(self):
        # Load model config
        yaml_config = _load_model_yaml(MODEL)

        # Joint names from ORCA-core config
        self.JOINT_NAMES = tuple(yaml_config["joint_ids"])

        # Build (N, 2) array of joint limits [radians], in order
        # The config uses 'joint_roms' which contains [lower, upper] in degrees for each joint
        jroms = yaml_config.get("joint_roms", {})
        self.JOINT_LIMITS = np.array([
            [
                np.deg2rad(jroms[j][0]),  # lower limit in degrees -> radians
                np.deg2rad(jroms[j][1])   # upper limit in degrees -> radians
            ] if j in jroms else [np.deg2rad(-10), np.deg2rad(90)]
            for j in self.JOINT_NAMES
        ], dtype=float)

        # Low-level ROS control bridge (talks to topics)
        self._control = DexArmControl()

        # Neutral pose (use neutral_position from config if available, otherwise zeros)
        neutral_config = yaml_config.get("neutral_position", {})
        self._home_pose = np.array([
            np.deg2rad(neutral_config.get(j, 0.0))
            for j in self.JOINT_NAMES
        ], dtype=float)

    # -----------------------------------------------------------------------
    # Open‑Teach robot interface
    # -----------------------------------------------------------------------

    def get_state(self) -> dict:
        """
        Return current joint positions from the ORCA hand.
        Format:
            { "position": np.ndarray of shape (N,) }
        """
        q = self._control.get_joint_angles()
        return {"position": q}

    def move_robot(self, joint_angles: np.ndarray) -> None:
        """
        Clamp and send a new joint command to the hand.
        """
        q_safe = self._clamp(joint_angles)
        self._control.send_joint_cmd(q_safe)

    def home_robot(self) -> None:
        """
        Move the hand to its neutral (zero) pose.
        """
        self.move_robot(self._home_pose)

    # -----------------------------------------------------------------------
    # Clamping utility, makes sure commands are within limits for safety
    # -----------------------------------------------------------------------

    def _clamp(self, q: np.ndarray) -> np.ndarray:
        """
        Clamp joint values to their respective physical limits.

        Returns:
            np.ndarray: clamped version of input joint vector
        """
        q_min, q_max = self.JOINT_LIMITS.T
        return np.clip(q, q_min, q_max)
