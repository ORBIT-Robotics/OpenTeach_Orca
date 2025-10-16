# ORCA Hand Teleoperation System Architecture

Complete system diagram showing how Meta Quest 3 hand tracking controls the ORCA robotic hand.

---

## 🎯 High-Level Overview

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────────┐
│   Meta Quest 3  │  WiFi   │  OpenTeach Host  │   ROS2  │   ORCA Hand     │
│  Hand Tracking  │────────→│   (Your PC)      │────────→│   Hardware      │
│   (21 points)   │   ZMQ   │   Python/Hydra   │  Topics │  (17 motors)    │
└─────────────────┘         └──────────────────┘         └─────────────────┘
      Human                  Control Pipeline              Physical Robot
```

---

## 📡 Complete Data Flow (End-to-End)

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 1: VR HAND TRACKING (Quest Headset)                                    │
└──────────────────────────────────────────────────────────────────────────────┘

    👋 Human Hand
         │
         │ (MediaPipe ML)
         ↓
    Quest 3 Hand Tracking
         │ - 21 landmarks per hand
         │ - 90 Hz update rate
         │ - World coordinates (x,y,z in meters)
         ↓
    Unity/Quest App
         │ - Runs on headset
         │ - ZMQ Publisher
         │
         │ ZMQ Socket: tcp://<host_address>:8087
         ↓

┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 2: NETWORK TRANSPORT (ZMQ over WiFi)                                   │
└──────────────────────────────────────────────────────────────────────────────┘

    Network Interface
         │ host_address: 172.24.71.206 (from configs/network.yaml)
         │ Protocol: ZeroMQ
         │ Data: 21x3 float array (63 values)
         ↓

┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 3: OPENTEACH DETECTION (Python Process #1)                             │
└──────────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────┐
    │ OculusVRHandDetector                                │
    │ File: openteach/components/detector/oculus.py       │
    │                                                     │
    │ Input:  Quest raw hand data (port 8087)            │
    │ Output: Published keypoints (port 8088)            │
    │                                                     │
    │ - Receives from Quest                              │
    │ - Validates data                                   │
    │ - Republishes for OpenTeach ecosystem              │
    └─────────────────────────────────────────────────────┘
         │
         │ ZMQ topic: 'hand_coords'
         │ Port: 8088
         ↓

┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 4: COORDINATE TRANSFORMATION (Python Process #2)                       │
└──────────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────┐
    │ TransformHandPositionCoords                         │
    │ File: openteach/components/detector/                │
    │       keypoint_transform.py                         │
    │                                                     │
    │ Input:  Raw keypoints (port 8088)                  │
    │ Output: Transformed keypoints (port 8089)          │
    │                                                     │
    │ - Coordinate system alignment                      │
    │ - Moving average filter (smooth noise)             │
    │ - Workspace bounds checking                        │
    │ - Optional scaling/offset                          │
    └─────────────────────────────────────────────────────┘
         │
         │ ZMQ topic: 'transformed_hand_coords'
         │ Port: 8089
         │
         ├─────────────────────────────────┐
         │                                 │
         ↓                                 ↓
    [Visualizer]                    [Operator]
    (Optional)                       (Main)

┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 5a: VISUALIZATION (Python Process #3 - Optional)                       │
└──────────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────┐
    │ Hand2DVisualizer                                    │
    │ File: openteach/components/visualizers/             │
    │       visualizer_2d.py                              │
    │                                                     │
    │ - Subscribes to port 8089                          │
    │ - Draws 2D hand skeleton with matplotlib           │
    │ - Saves plot to PNG                                │
    │ - Streams back to Quest (port 15001)               │
    │                                                     │
    │ Enable: visualize_right_2d: true in teleop.yaml   │
    └─────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 5b: RETARGETING & CONTROL (Python Process #4 - Main)                   │
└──────────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────────────────┐
    │ OrcaOperator (Main Control Loop - 90 Hz)                                │
    │ File: openteach/components/operators/orca_operator.py                   │
    │                                                                         │
    │ ┌─────────────────────────────────────────────────────────────────┐   │
    │ │ __init__():                                                      │   │
    │ │  - ZMQKeypointSubscriber(port 8089) → Quest data                │   │
    │ │  - OrcaHand() → Robot wrapper                                   │   │
    │ │  - FrequencyTimer(90 Hz) → Timing                               │   │
    │ │  - Config: handedness, thumb_dip_scale, gains, bias             │   │
    │ └─────────────────────────────────────────────────────────────────┘   │
    │                                                                         │
    │ ┌─────────────────────────────────────────────────────────────────┐   │
    │ │ stream() loop (from Operator base class):                       │   │
    │ │                                                                  │   │
    │ │  while True:                                                     │   │
    │ │      # Safety check                                             │   │
    │ │      if robot.get_joint_position() is not None:                 │   │
    │ │                                                                  │   │
    │ │          timer.start_loop()                                     │   │
    │ │                                                                  │   │
    │ │          # Core retargeting                                     │   │
    │ │          _apply_retargeted_angles()  ───────┐                   │   │
    │ │                                              │                   │   │
    │ │          timer.end_loop()  # Sleep to 90Hz  │                   │   │
    │ └──────────────────────────────────────────────┼──────────────────┘   │
    │                                                 │                       │
    │ ┌───────────────────────────────────────────────▼──────────────────┐   │
    │ │ _apply_retargeted_angles():                                      │   │
    │ │                                                                  │   │
    │ │  1. hand_keypoints = subscriber.recv_keypoints()                │   │
    │ │     └─ Receive 21x3 array from ZMQ                              │   │
    │ │                                                                  │   │
    │ │  2. joint_angles = quest21_to_orca17(                           │   │
    │ │         kp_world=hand_keypoints,                                │   │
    │ │         orca_joint_order=robot.JOINT_NAMES,                     │   │
    │ │         handedness=self.handedness,                             │   │
    │ │         thumb_dip_scale=self.thumb_dip_scale,                   │   │
    │ │         joint_gain=self.joint_gain,                             │   │
    │ │         joint_bias=self.joint_bias                              │   │
    │ │     )  ──────────────────────────────────┐                      │   │
    │ │                                           │                      │   │
    │ │  3. robot.move_robot(joint_angles)       │                      │   │
    │ │     └─ Send to ORCA hand                 │                      │   │
    │ └──────────────────────────────────────────┼──────────────────────┘   │
    └─────────────────────────────────────────────┼────────────────────────┘
                                                  │
    ┌─────────────────────────────────────────────▼────────────────────────┐
    │ quest21_to_orca17() - Geometric Retargeting                          │
    │                                                                      │
    │  Step 1: Build Palm Frame (rotation-invariant)                      │
    │  ┌────────────────────────────────────────────────────────────┐    │
    │  │ _to_palm_frame(kp_world):                                  │    │
    │  │  - wrist = kp[0]                                           │    │
    │  │  - xhat = normalize(index_mcp - pinky_mcp)  # Across palm │    │
    │  │  - yhat = normalize(middle_mcp - palm_center) # To fingers│    │
    │  │  - zhat = cross(xhat, yhat)  # Palm normal                │    │
    │  │  - Rotate all 21 points into palm frame                   │    │
    │  │  → Returns: (kp_palm, xhat, yhat, zhat)                   │    │
    │  └────────────────────────────────────────────────────────────┘    │
    │                                                                      │
    │  Step 2: Compute Joint Angles                                       │
    │  ┌────────────────────────────────────────────────────────────┐    │
    │  │ _compute_angles_in_palm_frame(p, xhat, yhat, zhat):        │    │
    │  │                                                             │    │
    │  │  For each finger (thumb, index, middle, ring, pinky):      │    │
    │  │                                                             │    │
    │  │    FLEXION (MCP, PIP):                                     │    │
    │  │      angle = arccos(dot(v1, v2))  # Interior angle         │    │
    │  │      where v1 = proximal bone, v2 = distal bone            │    │
    │  │                                                             │    │
    │  │    ABDUCTION:                                              │    │
    │  │      - Project finger onto palm plane (remove z)           │    │
    │  │      - Signed angle from xhat (palm width axis)            │    │
    │  │      - Negate if left hand                                 │    │
    │  │                                                             │    │
    │  │    WRIST (enable_wrist=true):                              │    │
    │  │      - Measure palm yhat tilt vs gravity                   │    │
    │  │      - arcsin(dot(yhat, -gravity))                         │    │
    │  │      - Positive = flexion (down), Negative = extension (up)│    │
    │  │                                                             │    │
    │  │  → Returns: dict of 17 angles keyed by joint name          │    │
    │  └────────────────────────────────────────────────────────────┘    │
    │                                                                      │
    │  Step 3: Pack & Calibrate                                           │
    │  ┌────────────────────────────────────────────────────────────┐    │
    │  │  for joint_name in orca_joint_order:                       │    │
    │  │      angle = angles_dict[joint_name]                       │    │
    │  │      if joint_name in joint_gain:                          │    │
    │  │          angle *= joint_gain[joint_name]                   │    │
    │  │      if joint_name in joint_bias:                          │    │
    │  │          angle += joint_bias[joint_name]                   │    │
    │  │      out.append(angle)                                     │    │
    │  │                                                             │    │
    │  │  → Returns: np.array of 17 angles (radians)                │    │
    │  └────────────────────────────────────────────────────────────┘    │
    └──────────────────────────────────────────────────────────────────────┘
         │
         │ 17 joint angles (radians)
         ↓

┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 6: ROBOT WRAPPER (OpenTeach Interface)                                 │
└──────────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────────────┐
    │ OrcaHand (RobotWrapper)                                             │
    │ File: openteach/robot/orca.py                                       │
    │                                                                     │
    │ ┌─────────────────────────────────────────────────────────────┐   │
    │ │ __init__():                                                  │   │
    │ │  - Load ORCA config YAML (joint_ids, joint_roms, neutral)   │   │
    │ │  - JOINT_NAMES = 17 joint names                             │   │
    │ │  - JOINT_LIMITS = [-50°,+30°] for wrist, etc (deg→rad)      │   │
    │ │  - _control = DexArmControl() → ROS bridge                  │   │
    │ │  - _home_pose = neutral position from config                │   │
    │ └─────────────────────────────────────────────────────────────┘   │
    │                                                                     │
    │ ┌─────────────────────────────────────────────────────────────┐   │
    │ │ move_robot(joint_angles):  ← Called at 90 Hz                │   │
    │ │  1. q_safe = _clamp(joint_angles)  # Enforce limits         │   │
    │ │  2. _control.send_joint_cmd(q_safe)  # To ROS bridge        │   │
    │ │  3. _last_cmd = q_safe  # Store for recording               │   │
    │ └─────────────────────────────────────────────────────────────┘   │
    │                                                                     │
    │ ┌─────────────────────────────────────────────────────────────┐   │
    │ │ get_joint_position():  ← Safety check before each loop      │   │
    │ │  return _control.get_joint_angles()                         │   │
    │ └─────────────────────────────────────────────────────────────┘   │
    │                                                                     │
    │ Other methods (for data collection, not active during teleop):     │
    │  - get_joint_state() → {'position': angles}                       │
    │  - get_commanded_joint_state() → {'position': _last_cmd}          │
    │  - home() → move to neutral                                       │
    │  - recorder_functions → dict for data collection                  │
    └─────────────────────────────────────────────────────────────────────┘
         │
         │ Commands via ROS bridge
         ↓

┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 7: ROS 2 BRIDGE (Thread-Safe Communication)                            │
└──────────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────────────┐
    │ DexArmControl (ROS 2 Node)                                          │
    │ File: openteach/ros_links/orca_control.py                           │
    │                                                                     │
    │ ROS 2 Topics:                                                       │
    │  - SUBSCRIBE: /orca_hand/joint_states (sensor_msgs/JointState)     │
    │  - PUBLISH:   /orca_hand/command (std_msgs/Float64MultiArray)      │
    │                                                                     │
    │ ┌─────────────────────────────────────────────────────────────┐   │
    │ │ __init__():                                                  │   │
    │ │  - rclpy.Node()                                             │   │
    │ │  - _joint_state = None  # Thread-safe state                 │   │
    │ │  - _state_lock = threading.Lock()                           │   │
    │ │  - subscription = create_subscription(...)                  │   │
    │ │  - publisher = create_publisher(...)                        │   │
    │ │  - Spin in background thread                                │   │
    │ └─────────────────────────────────────────────────────────────┘   │
    │                                                                     │
    │ ┌─────────────────────────────────────────────────────────────┐   │
    │ │ send_joint_cmd(q):  ← 90 Hz from operator                   │   │
    │ │  msg = Float64MultiArray()                                  │   │
    │ │  msg.data = q.tolist()                                      │   │
    │ │  self._publisher.publish(msg)                               │   │
    │ │  → Publishes to /orca_hand/command                          │   │
    │ └─────────────────────────────────────────────────────────────┘   │
    │                                                                     │
    │ ┌─────────────────────────────────────────────────────────────┐   │
    │ │ get_joint_angles():  ← Safety check                         │   │
    │ │  with self._state_lock:                                     │   │
    │ │      if self._joint_state is None:                          │   │
    │ │          return None  # Robot not connected                 │   │
    │ │      return self._joint_state.position                      │   │
    │ └─────────────────────────────────────────────────────────────┘   │
    │                                                                     │
    │ ┌─────────────────────────────────────────────────────────────┐   │
    │ │ _joint_state_callback(msg):  ← ROS subscription             │   │
    │ │  with self._state_lock:                                     │   │
    │ │      self._joint_state = msg  # Update from robot           │   │
    │ └─────────────────────────────────────────────────────────────┘   │
    └─────────────────────────────────────────────────────────────────────┘
         │
         │ ROS 2 DDS Middleware
         ↓

┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 8: ORCA HAND HARDWARE                                                  │
└──────────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────────────┐
    │ ORCA Hand ROS 2 Node (C++/Python on robot embedded PC)             │
    │                                                                     │
    │ Subscribes: /orca_hand/command                                     │
    │  → Receives 17 joint angle targets (radians)                       │
    │  → Converts to motor commands                                      │
    │  → Sends to motor controllers via CAN/USB                          │
    │                                                                     │
    │ Publishes: /orca_hand/joint_states                                 │
    │  → Reads encoder positions from 17 motors                          │
    │  → Publishes current state at ~100 Hz                              │
    │                                                                     │
    │ Motors (17 DOF):                                                   │
    │  - thumb_mcp, thumb_abd, thumb_pip, thumb_dip                      │
    │  - index_abd, index_mcp, index_pip                                 │
    │  - middle_abd, middle_mcp, middle_pip                              │
    │  - ring_abd, ring_mcp, ring_pip                                    │
    │  - pinky_abd, pinky_mcp, pinky_pip                                 │
    │  - wrist (flexion/extension)                                       │
    │                                                                     │
    │ Safety:                                                            │
    │  - Joint limit enforcement (hardware level)                        │
    │  - Torque limiting                                                 │
    │  - Emergency stop handling                                         │
    └─────────────────────────────────────────────────────────────────────┘
         │
         ↓
    🤖 Physical ORCA Hand Moves!

```

---

## 🗂️ File Structure

```
OpenTeach_Orca/
│
├── configs/
│   ├── network.yaml              # Network ports and IPs
│   ├── teleop.yaml              # Main config (selects robot)
│   └── robot/
│       └── orca.yaml            # ORCA-specific config (Hydra format)
│
├── openteach/
│   ├── components/
│   │   ├── detector/
│   │   │   └── oculus.py        # Layer 3: OculusVRHandDetector
│   │   │   └── keypoint_transform.py  # Layer 4: Transform coords
│   │   │
│   │   ├── operators/
│   │   │   ├── operator.py      # Base class with stream() loop
│   │   │   └── orca_operator.py # Layer 5: OrcaOperator + retargeting
│   │   │
│   │   └── visualizers/
│   │       └── visualizer_2d.py # Layer 5a: Hand2DVisualizer
│   │
│   ├── robot/
│   │   ├── robot.py             # RobotWrapper base class (interface)
│   │   └── orca.py              # Layer 6: OrcaHand wrapper
│   │
│   ├── ros_links/
│   │   └── orca_control.py      # Layer 7: DexArmControl (ROS bridge)
│   │
│   └── utils/
│       ├── network.py           # ZMQ subscribers/publishers
│       └── timer.py             # FrequencyTimer (90 Hz enforcement)
│
├── orca_core/                   # ORCA-core submodule
│   └── orca_core/
│       └── models/
│           └── orcahand_v1_right/
│               └── config.yaml  # Joint IDs, limits, neutral pose
│
└── teleop.py                    # Main entry point (Hydra launcher)
```

---

## ⚙️ Configuration Flow (Hydra)

```
python teleop.py robot=orca
    ↓
configs/teleop.yaml
    defaults:
      - robot: orca  ← Loads configs/robot/orca.yaml
    ↓
configs/robot/orca.yaml
    detector:
      _target_: openteach.components.detector.oculus.OculusVRHandDetector
      ↓ Hydra instantiates with params
    
    transforms:
      _target_: ...TransformHandPositionCoords
      ↓ Hydra instantiates
    
    operators:
      _target_: openteach.components.operators.orca_operator.OrcaOperator
      config:
        handedness: right
        thumb_dip_scale: 0.3
        enable_wrist: true
        joint_gain: {}
        joint_bias: {}
      ↓ Hydra instantiates with config dict
    
    controllers:
      _target_: openteach.robot.orca.OrcaHand
      ↓ Hydra instantiates
    ↓
All processes started as multiprocessing.Process
Each runs in separate Python process
```

---

## 🔄 Timing & Frequencies

| Component | Frequency | Why |
|-----------|-----------|-----|
| Quest Hand Tracking | 90 Hz | Native Quest 3 tracking rate |
| OculusVRHandDetector | 90 Hz | Matches Quest output |
| TransformHandPositionCoords | 90 Hz | Pass-through with filtering |
| OrcaOperator Loop | 90 Hz | Matches VR_FREQ constant |
| Hand2DVisualizer | 90 Hz | Real-time feedback |
| ROS /orca_hand/command | 90 Hz | Published by DexArmControl |
| ROS /orca_hand/joint_states | 100 Hz | Published by ORCA hardware |

**Critical:** `FrequencyTimer` enforces 90 Hz by sleeping remainder of 11ms period.

---

## 🔒 Safety Mechanisms

1. **Joint Clamping** (Software)
   - `OrcaHand._clamp()` enforces limits before sending
   - Limits loaded from ORCA-core YAML (e.g., wrist: -50° to +30°)

2. **Robot Alive Check**
   ```python
   if robot.get_joint_position() is not None:  # Check every loop
   ```
   - Returns `None` if no ROS messages received
   - Skips command if robot disconnected

3. **Thread-Safe ROS Bridge**
   - `DexArmControl` uses locks for `_joint_state` access
   - Prevents race conditions between ROS callback and main thread

4. **Hardware-Level Limits**
   - ORCA hand firmware enforces limits
   - Torque limiting to prevent damage

---

## 📊 Data Types

| Stage | Format | Example |
|-------|--------|---------|
| Quest → ZMQ | `float[21][3]` | `[[0.0, 0.5, 0.1], [0.02, 0.48, ...], ...]` |
| Detector → Transform | `np.ndarray (21,3)` | Same, as NumPy |
| Transform → Operator | `np.ndarray (21,3)` | Smoothed coordinates |
| Retargeting Output | `np.ndarray (17,)` | `[0.5, 0.3, 0.8, 0.0, ...]` radians |
| ROS Command | `Float64MultiArray` | `{data: [0.5, 0.3, ...]}` |
| ROS Feedback | `JointState` | `{position: [...], velocity: [...]}` |

---

## 🎮 Control Modes

### Teleoperation Mode (Active)
```bash
python teleop.py robot=orca
```
- Uses: `get_joint_position()`, `move_robot()`
- Frequency: 90 Hz
- Purpose: Real-time control

### Data Collection Mode
```bash
python data_collect.py robot=orca
```
- Uses: `recorder_functions`, `get_joint_state()`, `get_commanded_joint_state()`
- Frequency: 90 Hz recording
- Purpose: Capture demonstrations for learning

### Visualization Mode
```yaml
# In configs/teleop.yaml
visualize_right_2d: true
```
- Runs Hand2DVisualizer in parallel
- Shows hand skeleton plot
- Streams back to Quest at port 15001

---

## 🧩 Key Design Patterns

1. **Multiprocessing Architecture**
   - Each component (detector, transform, operator, visualizer) = separate process
   - Decoupled via ZMQ pub/sub
   - Can run on different machines

2. **Hydra Dependency Injection**
   - Components instantiated from config `_target_` paths
   - No hardcoded class imports in main script
   - Easy to swap robots by changing `robot=orca` argument

3. **Abstract Base Classes**
   - `Operator`: Forces implementation of `_apply_retargeted_angles()`
   - `RobotWrapper`: Enforces standard interface across robots
   - Enables polymorphism (same operator code for different robots)

4. **Geometric Retargeting**
   - No machine learning required
   - Rotation-invariant (palm-frame based)
   - Real-time capable (pure NumPy math)

5. **Configuration Hierarchy**
   - Network ports: `network.yaml`
   - Robot-specific: `robot/orca.yaml`
   - Main settings: `teleop.yaml`
   - ORCA hardware: `orca_core/.../config.yaml`

---

## 🐛 Debugging Entry Points

**Check Quest Connection:**
```bash
# Test if Quest is publishing
python -c "from openteach.utils.network import ZMQKeypointSubscriber; \
           s = ZMQKeypointSubscriber('172.24.71.206', 8087, 'hand_coords'); \
           print(s.recv_keypoints())"
```

**Check ROS Connection:**
```bash
# See if ORCA is publishing joint states
ros2 topic echo /orca_hand/joint_states
```

**Check Retargeting:**
```python
# Test geometric conversion
from openteach.components.operators.orca_operator import quest21_to_orca17
import numpy as np
keypoints = np.random.rand(21, 3)  # Fake Quest data
angles = quest21_to_orca17(keypoints, joint_order=("thumb_mcp", ...))
print(angles)  # Should be 17 values in radians
```

---

## 📈 Performance Metrics

**Expected Latencies:**
- Quest tracking: <5ms (native MediaPipe)
- Network (WiFi): ~10-20ms
- ZMQ transport: ~1ms per hop
- Retargeting math: <1ms (NumPy on modern CPU)
- ROS publish: ~1ms
- Total end-to-end: **~30-50ms** (Quest hand → ORCA movement)

**At 90 Hz:**
- Loop period: 11.1ms
- Compute budget: ~10ms (1ms margin for jitter)

---

This schema shows the complete architecture from human hand gesture to robot motion. The system is modular, real-time capable, and follows OpenTeach conventions for easy integration with data collection and policy learning pipelines. 🎯
