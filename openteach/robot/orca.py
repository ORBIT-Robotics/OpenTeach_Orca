"""
orca.py
=======
Robot wrapper for the ORCA robotic hand.

This class provides the standard robot API expected by Open‑Teach:
    - get_state()
    - move_robot()
    - home_robot()

Instead of hard‑coding joints, it *auto‑loads* the correct joint names
and limits directly from the ORCA‑core model YAML file, ensuring it is
always up‑to‑date with the hardware configuration.

The rest of Open‑Teach (operators, teleop, etc.) can then stay generic.
"""

import importlib.resources as pkg
from pathlib import Path
import yaml
import numpy as np

# Low‑level ROS communication bridge
from openteach.ros_links.orca_control import DexArmControl

# ORCA‑core must be installed (pip install -e ./orca_core)
import orca_core


# ---------------------------------------------------------------------------
# Model selection
# ---------------------------------------------------------------------------
# Change this to 'orcahand_v1_left' or another version as needed
MODEL = "orcahand_v1_right"


def _load_model_yaml(model: str) -> dict:
    """
    Load the YAML configuration file for the selected ORCA hand model.
    """
    cfg_path = Path(pkg.files(orca_core)) / f"models/{model}/config.yaml"
    return yaml.safe_load(cfg_path.read_text())


# ---------------------------------------------------------------------------
# Main wrapper class
# ---------------------------------------------------------------------------

class OrcaHand:
    """
    Open‑Teach robot wrapper for a single ORCA hand.
    Auto-syncs with ORCA‑core's YAML and delegates ROS communication
    to the DexArmControl class.
    """

    # Load model config
    _yaml = _load_model_yaml(MODEL)

    # Joint names from ORCA-core config
    JOINT_NAMES = tuple(_yaml["joint_ids"])

    # Build (N, 2) array of joint limits [rad], in order
    _jl = _yaml.get("joint_limits", {})
    JOINT_LIMITS = np.array([
        [
            np.deg2rad(_jl[j]["lower"]),
            np.deg2rad(_jl[j]["upper"])
        ] if j in _jl else [np.deg2rad(-10), np.deg2rad(90)]
        for j in JOINT_NAMES
    ], dtype=float)

    def __init__(self):
        # Low-level ROS control bridge (talks to topics)
        self._control = DexArmControl()

        # Neutral pose (0 rad for all joints)
        self._home_pose = np.zeros(len(self.JOINT_NAMES))

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
    # Helper
    # -----------------------------------------------------------------------

    def _clamp(self, q: np.ndarray) -> np.ndarray:
        """
        Clamp joint values to their respective physical limits.

        Returns:
            np.ndarray: clamped version of input joint vector
        """
        q_min, q_max = self.JOINT_LIMITS.T
        return np.clip(q, q_min, q_max)
