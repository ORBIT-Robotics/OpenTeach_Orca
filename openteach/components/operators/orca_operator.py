"""
orca_operator.py
================
OpenTeach operator that connects Meta Quest 3 hand tracking to the ORCA Hand.

Pipeline (rotation-invariant):
1) Build a palm-local frame from Quest keypoints (wrist + MCPs).
2) Express all 21 points in that frame (remove camera motion).
3) Compute geometric joint angles:
   - MCP/PIP flexion via interior angles between adjacent bones.
   - MCP abduction via angle of proximal phalanx projected on palm plane vs palm x-axis.
   - Thumb DIP approximated from IP (scale).
   - Wrist set to 0 (or estimate from palm tilt if desired).
4) Pack angles in the exact order expected by OrcaHand.JOINT_NAMES and send.

Architecture:
- Inherits from Operator (OpenTeach base class for all robot operators)
- Uses ZMQKeypointSubscriber for receiving hand tracking data from Quest
- Delegates to OrcaHand for robot communication (ROS 2 bridge)
- Implements geometric retargeting (no machine learning required)
"""

from __future__ import annotations
import numpy as np
from typing import Dict, Tuple

# OpenTeach base operator class and utilities
from openteach.components.operators.operator import Operator
from openteach.utils.network import ZMQKeypointSubscriber
from openteach.utils.timer import FrequencyTimer
from openteach.constants import VR_FREQ

# OrcaHand robot wrapper (contains ROS bridge + ORCA model metadata)
from openteach.robot.orca import OrcaHand


# -----------------------------
# Quest / MediaPipe landmarks
# -----------------------------
WRIST = 0
# Thumb
TH_CMC, TH_MCP, TH_IP, TH_TIP = 1, 2, 3, 4
# Index
IX_MCP, IX_PIP, IX_DIP, IX_TIP = 5, 6, 7, 8
# Middle
MD_MCP, MD_PIP, MD_DIP, MD_TIP = 9, 10, 11, 12
# Ring
RG_MCP, RG_PIP, RG_DIP, RG_TIP = 13, 14, 15, 16
# Pinky
PK_MCP, PK_PIP, PK_DIP, PK_TIP = 17, 18, 19, 20


# =========================
# Geometry helper functions
# =========================

def _normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / (n + 1e-9)


def _angle_at(B: np.ndarray, A: np.ndarray, C: np.ndarray) -> float:
    """
    Interior angle at B formed by segments BA and BC (in radians).
    A, B, C: (3,) points in the SAME (palm) frame.
    """
    
    v1 = _normalize(A - B)
    v2 = _normalize(C - B)
    cosang = float(np.clip(np.dot(v1, v2), -1.0, 1.0))
    return float(np.arccos(cosang))


def _project_to_plane(v: np.ndarray, n: np.ndarray) -> np.ndarray:
    """Project vector v onto plane with normal n (both (3,))."""
    return v - np.dot(v, n) * n


def _build_palm_frame(kp_world: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Build a palm-local orthonormal basis R = [xhat, yhat, zhat] and return (R, wrist_pos).
    kp_world:(21,3) array containing the 3D coordinates of the 21 hand keypoints provided by the MQ3 hand tracking system.
    xhat: across palm (pinky MCP -> index MCP)
    yhat: from palm center toward middle MCP (refined by orthogonalization)
    zhat: palm normal (xhat × yhat)
    """
    w = kp_world[WRIST] #wrist position
    i_mcp, p_mcp = kp_world[IX_MCP], kp_world[PK_MCP]
    m_mcp = kp_world[MD_MCP]

    xhat = _normalize(i_mcp - p_mcp)
    y0 = _normalize(m_mcp - 0.5 * (i_mcp + p_mcp))
    zhat = _normalize(np.cross(xhat, y0))
    yhat = _normalize(np.cross(zhat, xhat))

    R = np.stack([xhat, yhat, zhat], axis=1)  # columns are basis vectors
    return R, w


def _to_palm_frame(kp_world: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Translate by wrist and rotate to palm frame.
    Returns: (kp_palm, xhat, yhat, zhat)
    
    Note: xhat, yhat, zhat are returned in WORLD frame (before rotation)
    to allow wrist angle estimation relative to gravity.
    """
    R, w = _build_palm_frame(kp_world) 
    kp_centered = kp_world - w #translates to palm frame
    kp_palm = kp_centered @ R  # (21,3) * (3,3): world -> palm, rotates to palm frame
    xhat, yhat, zhat = R[:, 0], R[:, 1], R[:, 2]
    return kp_palm, xhat, yhat, zhat


def _estimate_wrist_flexion(
    yhat_world: np.ndarray,
    gravity: np.ndarray = np.array([0.0, -1.0, 0.0])
) -> float:
    """
    Estimate wrist flexion/extension angle from palm orientation.
    
    The ORCA wrist joint only does flexion/extension (1-DOF), not rotation.
    We estimate this by measuring the tilt of the palm y-axis (wrist→fingers)
    relative to gravity.
    
    Convention (ORCA wrist ROM: -50° to +30°):
    - 0° = neutral (palm y-axis horizontal, perpendicular to gravity)
    - Positive angle = flexion (palm tilts down, fingers point toward ground)
    - Negative angle = extension (palm tilts up, fingers point toward sky)
    
    Parameters
    ----------
    yhat_world : np.ndarray
        Palm y-axis in world coordinates (unit vector), shape (3,)
        Points from wrist toward middle MCP
    gravity : np.ndarray
        Gravity vector in world frame, shape (3,)
        Default: [0, -1, 0] assumes Y-down (Quest/OpenGL convention)
        Use [0, 0, -1] for Z-down coordinate systems
    
    Returns
    -------
    float
        Wrist flexion angle in radians
        Positive = flexion (hand bends down)
        Negative = extension (hand bends up)
    
    Notes
    -----
    - This assumes user holds hand in natural upright orientation
    - Works best when forearm is roughly horizontal
    - For more accuracy, track forearm orientation from Quest controller
    - ORCA will clamp to [-50°, +30°] automatically via joint limits
    """
    # Normalize gravity vector
    g_norm = _normalize(gravity)
    
    # Project palm y-axis onto gravity direction
    # When hand points down: yhat aligns with -gravity → positive value
    # When hand points up: yhat aligns with +gravity → negative value
    yhat_vertical_component = np.dot(yhat_world, g_norm)
    
    # Compute flexion angle using arcsin
    # Clamp to [-1, 1] for numerical stability
    wrist_flexion = float(np.arcsin(np.clip(-yhat_vertical_component, -1.0, 1.0)))
    
    return wrist_flexion


# ==========================================
# Main conversion: Quest21 -> ORCA17 angles
# ==========================================

def _compute_angles_in_palm_frame(
    p: np.ndarray,
    xhat: np.ndarray,
    yhat: np.ndarray,
    zhat: np.ndarray,
    handedness: str = "right",
    thumb_dip_scale: float = 0.3,
    enable_wrist: bool = True,
) -> Dict[str, float]:
    """
    Compute physically-meaningful joint angles from palm-frame keypoints.
    
    Parameters
    ----------
    p : np.ndarray
        21 keypoints in palm-local frame, shape (21, 3)
    xhat : np.ndarray
        Palm x-axis in WORLD frame (for wrist estimation), shape (3,)
    yhat : np.ndarray
        Palm y-axis in WORLD frame (for wrist estimation), shape (3,)
    zhat : np.ndarray
        Palm normal in WORLD frame (for abduction computation), shape (3,)
    handedness : str
        "right" or "left" - affects sign of abduction angles
    thumb_dip_scale : float
        Scaling factor to approximate thumb DIP from PIP (default 0.3)
    enable_wrist : bool
        If True, estimate wrist angle from palm tilt
        If False, set wrist=0 (useful for debugging)
    
    Returns
    -------
    Dict[str, float]
        Dictionary of joint angles keyed by ORCA joint names
        All angles in radians
    
    Notes
    -----
    - Flexion angles use interior angle formula (always positive)
    - Abduction angles use signed projection onto palm plane
    - Left hand abductions are negated for mirror symmetry
    - Wrist uses gravity-based tilt estimation (optional)
    """

    # ---- Thumb flexions ----
    thumb_mcp = _angle_at(p[TH_MCP], p[TH_CMC], p[TH_IP])
    thumb_pip = _angle_at(p[TH_IP],  p[TH_MCP], p[TH_TIP])
    thumb_dip = float(thumb_dip_scale * thumb_pip)  # No separate DIP landmark -> approximate

    # Thumb abduction: projection of proximal (MCP->IP) on palm plane vs x-axis
    th_prox_dir = _normalize(p[TH_IP] - p[TH_MCP])
    th_prox_proj = _project_to_plane(th_prox_dir, zhat)
    # signed angle from xhat to th_prox_proj around zhat:
    thumb_abd = float(np.arctan2(np.dot(np.cross(xhat, th_prox_proj), zhat),
                                 np.dot(xhat, th_prox_proj)))

    # ---- Index ----
    index_mcp = _angle_at(p[IX_MCP], p[WRIST],  p[IX_PIP])
    index_pip = _angle_at(p[IX_PIP], p[IX_MCP], p[IX_DIP])
    ix_dir = _normalize(p[IX_PIP] - p[IX_MCP])
    ix_proj = _project_to_plane(ix_dir, zhat)
    index_abd = float(np.arctan2(np.dot(np.cross(xhat, ix_proj), zhat),
                                 np.dot(xhat, ix_proj)))

    # ---- Middle ----
    middle_mcp = _angle_at(p[MD_MCP], p[WRIST],  p[MD_PIP])
    middle_pip = _angle_at(p[MD_PIP], p[MD_MCP], p[MD_DIP])
    md_dir = _normalize(p[MD_PIP] - p[MD_MCP])
    md_proj = _project_to_plane(md_dir, zhat)
    middle_abd = float(np.arctan2(np.dot(np.cross(xhat, md_proj), zhat),
                                  np.dot(xhat, md_proj)))

    # ---- Ring ----
    ring_mcp = _angle_at(p[RG_MCP], p[WRIST],  p[RG_PIP])
    ring_pip = _angle_at(p[RG_PIP], p[RG_MCP], p[RG_DIP])
    rg_dir = _normalize(p[RG_PIP] - p[RG_MCP])
    rg_proj = _project_to_plane(rg_dir, zhat)
    ring_abd = float(np.arctan2(np.dot(np.cross(xhat, rg_proj), zhat),
                                np.dot(xhat, rg_proj)))

    # ---- Pinky ----
    pinky_mcp = _angle_at(p[PK_MCP], p[WRIST],  p[PK_PIP])
    pinky_pip = _angle_at(p[PK_PIP], p[PK_MCP], p[PK_DIP])
    pk_dir = _normalize(p[PK_PIP] - p[PK_MCP])
    pk_proj = _project_to_plane(pk_dir, zhat)
    pinky_abd = float(np.arctan2(np.dot(np.cross(xhat, pk_proj), zhat),
                                 np.dot(xhat, pk_proj)))

    # ---- Wrist flexion/extension ----
    if enable_wrist:
        # Estimate from palm tilt relative to gravity
        # yhat points from wrist toward fingers
        wrist = _estimate_wrist_flexion(yhat)
    else:
        # Disabled - keep neutral
        wrist = 0.0

    # Left hand: flip abduction sign
    if handedness.lower().startswith("l"):
        thumb_abd, index_abd, middle_abd, ring_abd, pinky_abd = (
            -thumb_abd, -index_abd, -middle_abd, -ring_abd, -pinky_abd
        )

    angles = {
        # Thumb
        "thumb_mcp": thumb_mcp,
        "thumb_abd": thumb_abd,
        "thumb_pip": thumb_pip,
        "thumb_dip": thumb_dip,
        # Index
        "index_abd": index_abd,
        "index_mcp": index_mcp,
        "index_pip": index_pip,
        # Middle
        "middle_abd": middle_abd,
        "middle_mcp": middle_mcp,
        "middle_pip": middle_pip,
        # Ring
        "ring_abd": ring_abd,
        "ring_mcp": ring_mcp,
        "ring_pip": ring_pip,
        # Pinky
        "pinky_abd": pinky_abd,
        "pinky_mcp": pinky_mcp,
        "pinky_pip": pinky_pip,
        # Wrist
        "wrist": wrist,
    }
    return angles


def quest21_to_orca17(
    kp_world: np.ndarray,
    orca_joint_order: Tuple[str, ...],
    handedness: str = "right",
    thumb_dip_scale: float = 0.3,
    enable_wrist: bool = True,
    joint_gain: Dict[str, float] | None = None,
    joint_bias: Dict[str, float] | None = None,
) -> np.ndarray:
    """
    Convert 21 Quest keypoints (world coords) -> 17 ORCA angles (radians)
    ordered exactly as in `orca_joint_order` (OrcaHand.JOINT_NAMES).

    Parameters
    ----------
    kp_world : np.ndarray
        21 hand keypoints in world coordinates, shape (21, 3)
    orca_joint_order : Tuple[str, ...]
        Joint names in the order expected by ORCA (from OrcaHand.JOINT_NAMES)
    handedness : str
        "right" or "left" - affects abduction and wrist signs
    thumb_dip_scale : float
        Scaling factor for thumb DIP approximation (default: 0.3)
    enable_wrist : bool
        If True, estimate wrist angle from palm tilt (default: True)
        If False, wrist stays at 0 (useful for debugging)
    joint_gain : Dict[str, float] | None
        Optional per-joint multipliers applied AFTER geometry
        Example: {"index_mcp": 1.1, "thumb_abd": 0.8}
    joint_bias : Dict[str, float] | None
        Optional per-joint offsets (radians) applied AFTER gain
        Example: {"wrist": 0.1} adds 0.1 rad to wrist angle
    
    Returns
    -------
    np.ndarray
        17 joint angles in radians, ordered as orca_joint_order
        Shape: (17,), dtype: float32
    
    Notes
    -----
    - Geometric computation is rotation-invariant (uses palm frame)
    - Joint limits are enforced in OrcaHand.move_robot(), not here
    - Left hand support includes mirroring of abduction + wrist
    """
    assert kp_world.shape == (21, 3), f"Expected (21,3) keypoints, got {kp_world.shape}"

    # 1) Build palm frame and transform points
    # Note: xhat, yhat, zhat are in WORLD frame for wrist estimation
    p, xhat, yhat, zhat = _to_palm_frame(kp_world)

    # 2) Compute angles in a dict keyed by ORCA joint names
    angles_dict = _compute_angles_in_palm_frame(
        p=p, 
        xhat=xhat, 
        yhat=yhat,  # Now passed for wrist estimation
        zhat=zhat, 
        handedness=handedness, 
        thumb_dip_scale=thumb_dip_scale,
        enable_wrist=enable_wrist
    )

    # 3) Pack into array following the exact ORCA joint order
    out = []
    for name in orca_joint_order:
        a = angles_dict.get(name, 0.0)  # default 0 if not present (future-proof)
        
        # Apply optional calibration
        if joint_gain and name in joint_gain:
            a = joint_gain[name] * a
        if joint_bias and name in joint_bias:
            a = a + joint_bias[name]
        
        out.append(a)
    
    return np.array(out, dtype=np.float32)


# ============================
# Operator class implementation
# ============================

class OrcaOperator(Operator):
    """
    Operator for controlling the ORCA hand using Meta Quest 3 tracking.
    
    This class bridges Quest hand tracking data with the ORCA robotic hand by:
    1. Receiving 21 3D hand keypoints via ZMQ from the Quest headset
    2. Computing 17 joint angles using geometric retargeting (palm-frame based)
    3. Sending commands to the physical ORCA hand via ROS 2
    
    Inherits from OpenTeach's Operator base class which provides:
    - Abstract properties: robot, timer, subscribers
    - stream() method that runs the control loop
    - _apply_retargeted_angles() hook for retargeting logic
    """

    def __init__(self, host: str, transformed_keypoints_port: int, config: Dict):
        """
        Initialize the ORCA hand operator.
        
        Parameters
        ----------
        host : str
            IP address of the machine running the Quest hand tracking publisher
        transformed_keypoints_port : int
            ZMQ port for receiving transformed hand keypoints
        config : Dict
            Configuration dictionary containing:
              - handedness: "right" | "left" (default: "right")
              - thumb_dip_scale: float (default: 0.3) - scaling factor for thumb DIP joint
              - joint_gain: Dict[str, float] - optional per-joint multipliers
              - joint_bias: Dict[str, float] - optional per-joint offsets (radians)
        """
        self.notify_component_start('orca hand operator')
        
        # Store network configuration
        self._host = host
        self._port = transformed_keypoints_port
        
        # Initialize ZMQ subscriber for hand keypoints from Quest
        # Topic matches what the Quest tracking publisher sends
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=self._host,
            port=self._port,
            topic='transformed_hand_coords'  # 21x3 hand keypoints
        )
        
        # Initialize ZMQ subscriber for arm/hand frame (orientation reference)
        # Note: Currently not used for pure hand control, but required by Operator interface
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=self._host,
            port=self._port,
            topic='transformed_hand_frame'
        )

        # Initialize the ORCA hand robot wrapper
        # This handles:
        # - Loading joint limits/names from orca_core YAML
        # - ROS 2 communication via DexArmControl
        # - Joint clamping for safety
        self._robot = OrcaHand()

        # Initialize frequency timer for consistent control rate
        self._timer = FrequencyTimer(VR_FREQ)  # Typically 90 Hz for VR

        # ---- Retargeting configuration ----
        self.config = config
        self.handedness = str(self.config.get("handedness", "right"))
        self.thumb_dip_scale = float(self.config.get("thumb_dip_scale", 0.3))
        
        # Optional per-joint calibration (applied after geometric computation)
        # Example: {"index_mcp": 1.1, "thumb_abd": 0.8}
        self.joint_gain: Dict[str, float] = dict(self.config.get("joint_gain", {}))
        self.joint_bias: Dict[str, float] = dict(self.config.get("joint_bias", {}))

    # ---- Required Operator interface properties ----
    
    @property
    def timer(self):
        """Return the frequency timer for control loop timing."""
        return self._timer

    @property
    def robot(self):
        """Return the ORCA hand robot instance."""
        return self._robot

    @property
    def transformed_hand_keypoint_subscriber(self):
        """Return the hand keypoint subscriber."""
        return self._transformed_hand_keypoint_subscriber
    
    @property
    def transformed_arm_keypoint_subscriber(self):
        """Return the arm/frame subscriber."""
        return self._transformed_arm_keypoint_subscriber
    
    # ---- Helper method ----
    
    def return_real(self):
        """
        Differentiate between real robot and simulation.
        ORCA is always real hardware (no sim support yet).
        """
        return True

    # ---- Main retargeting logic ----
    
    def _apply_retargeted_angles(self):
        """
        Core retargeting method called by stream() loop.
        
        Flow:
        1. Receive 21x3 hand keypoints from Quest via ZMQ
        2. Compute 17 ORCA joint angles via geometric retargeting
        3. Send angles to robot (clamped to limits internally)
        
        This is called at VR_FREQ (typically 90 Hz).
        """
        # Receive latest hand keypoints from Quest (21 landmarks, 3D coords)
        # Format: numpy array of shape (21, 3) in world coordinates
        hand_keypoints = self._transformed_hand_keypoint_subscriber.recv_keypoints()

        # Convert Quest's 21 keypoints -> ORCA's 17 joint angles
        # Uses palm-frame geometric retargeting (rotation-invariant)
        joint_angles = quest21_to_orca17(
            kp_world=hand_keypoints,
            orca_joint_order=self.robot.JOINT_NAMES,  # Ensures correct ordering
            handedness=self.handedness,
            thumb_dip_scale=self.thumb_dip_scale,
            joint_gain=self.joint_gain,
            joint_bias=self.joint_bias,
        )

        # Send to robot (safety clamping handled in OrcaHand.move_robot)
        self.robot.move_robot(joint_angles)
