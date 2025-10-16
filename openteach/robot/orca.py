"""
orca.py
=======
OpenTeach robot wrapper for the ORCA Hand.

This module provides the high-level interface to the ORCA hand hardware.
It auto-loads joint names and limits from the ORCA-core YAML file and
delegates low-level ROS 2 communication to the DexArmControl class.

Usage:
    from openteach.robot.orca import OrcaHand
    
    hand = OrcaHand()
    hand.home()
    hand.move(some_joint_angles)
    current_state = hand.get_joint_state()
"""

from __future__ import annotations
import numpy as np
from pathlib import Path
from typing import Dict, Callable

# OpenTeach base class
from openteach.robot.robot import RobotWrapper

# ROS 2 bridge
from openteach.ros_links.orca_control import DexArmControl

# ORCA core integration
try:
    import yaml
except ImportError:
    raise ImportError("pyyaml is required. Install with: pip install pyyaml")


# ============================
# Constants
# ============================

# Path to the ORCA hand model within orca_core submodule
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

class OrcaHand(RobotWrapper):
    """
    OpenTeach robot wrapper for a single ORCA hand.
    
    Inherits from RobotWrapper to provide the standard OpenTeach interface
    for robot control, state monitoring, and data recording.
    
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
        
        # Data recording frequency (Hz) - matches VR_FREQ for hand tracking
        self._data_frequency = 90

    # -----------------------------------------------------------------------
    # RobotWrapper required properties
    # -----------------------------------------------------------------------
    
    @property
    def name(self) -> str:
        """Robot identifier for logging and config."""
        return "orca_hand"
    
    @property
    def recorder_functions(self) -> Dict[str, Callable]:
        """
        Mapping of data keys to getter functions for recording.
        
        Returns dict with keys matching the 'recorded_data' section
        in configs/robot/orca.yaml:
            - joint_states: full state dict with positions
            - commanded_joint_states: last commanded angles
        """
        return {
            'joint_states': self.get_joint_state,
            'commanded_joint_states': self.get_commanded_joint_state,
        }
    
    @property
    def data_frequency(self) -> int:
        """Recording frequency in Hz (matches VR tracking rate)."""
        return self._data_frequency

    # -----------------------------------------------------------------------
    # RobotWrapper required methods - State queries
    # -----------------------------------------------------------------------

    def get_joint_state(self) -> Dict[str, np.ndarray]:
        """
        Return current joint state from the ORCA hand.
        
        Returns:
            dict with keys:
                - 'position': np.ndarray of shape (17,) in radians
                - 'velocity': np.ndarray of shape (17,) in rad/s (if available)
                - 'effort': np.ndarray of shape (17,) in Nm (if available)
        """
        q = self._control.get_joint_angles()
        state = {"position": q}
        
        # Add velocity/effort if ROS bridge provides them
        # (DexArmControl currently only provides positions)
        # state["velocity"] = np.zeros_like(q)
        # state["effort"] = np.zeros_like(q)
        
        return state

    def get_joint_position(self) -> np.ndarray:
        """
        Return just the joint positions (for compatibility).
        
        Returns:
            np.ndarray of shape (17,) in radians
        """
        return self._control.get_joint_angles()
    
    def get_commanded_joint_state(self) -> Dict[str, np.ndarray]:
        """
        Return the last commanded joint angles.
        
        Useful for recording what was sent to the robot vs actual state.
        
        Returns:
            dict with 'position' key containing last commanded angles
        """
        # Store last command in instance variable (updated in move())
        if not hasattr(self, '_last_cmd'):
            self._last_cmd = self._home_pose.copy()
        return {"position": self._last_cmd}

    def get_cartesian_position(self) -> np.ndarray:
        """
        Get end-effector Cartesian position (not applicable for hand-only robot).
        
        Returns:
            Empty array (ORCA hand has no single end-effector, only fingertips)
        """
        # Hand-only robot - no arm kinematics
        # Could return fingertip positions from forward kinematics if needed
        return np.array([])

    def get_joint_velocity(self) -> np.ndarray:
        """Get joint velocities (not currently available from ROS bridge)."""
        return np.zeros(len(self.JOINT_NAMES))

    def get_joint_torque(self) -> np.ndarray:
        """Get joint torques (not currently available from ROS bridge)."""
        return np.zeros(len(self.JOINT_NAMES))

    # -----------------------------------------------------------------------
    # RobotWrapper required methods - Commands
    # -----------------------------------------------------------------------

    def home(self) -> None:
        """
        Move the hand to its neutral pose (as defined in ORCA config).
        
        This is the RobotWrapper standard method name.
        """
        self.move(self._home_pose)

    def move(self, input_angles: np.ndarray) -> None:
        """
        Send joint position command to the hand (with safety clamping).
        
        This is the RobotWrapper standard method name.
        
        Args:
            input_angles: np.ndarray of shape (17,) in radians
        """
        q_safe = self._clamp(input_angles)
        self._control.send_joint_cmd(q_safe)
        
        # Store for recording
        self._last_cmd = q_safe.copy()

    def move_coords(self, input_coords: np.ndarray) -> None:
        """
        Move to Cartesian coordinates (not applicable for hand-only robot).
        
        Args:
            input_coords: Cartesian pose (ignored for ORCA hand)
        
        Note:
            ORCA hand has no arm, so Cartesian control is not supported.
            This is a stub to satisfy the RobotWrapper interface.
        """
        # Hand-only robot - no Cartesian control
        # Could implement inverse kinematics for fingertip positions if needed
        pass
    
    # -----------------------------------------------------------------------
    # Backward compatibility aliases (for existing operator code)
    # -----------------------------------------------------------------------
    
    def get_state(self) -> Dict[str, np.ndarray]:
        """Alias for get_joint_state() for backward compatibility."""
        return self.get_joint_state()
    
    def move_robot(self, joint_angles: np.ndarray) -> None:
        """Alias for move() for backward compatibility."""
        self.move(joint_angles)
    
    def home_robot(self) -> None:
        """Alias for home() for backward compatibility."""
        self.home()

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
