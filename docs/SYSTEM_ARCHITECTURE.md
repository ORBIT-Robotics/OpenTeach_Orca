# ORCA Hand Teleoperation - System Architecture & Reference

Complete technical reference for Meta Quest 3 → ORCA Hand teleoperation system.

---

## 🎯 Quick Overview

```
👋 Human Hand
    ↓ MediaPipe (90 Hz)
🥽 Quest 3 Tracking (21 points)
    ↓ WiFi/ZMQ (port 8087)
🖥️  PC: OculusVRHandDetector
    ↓ ZMQ (port 8088)
🖥️  PC: TransformHandPositionCoords
    ↓ ZMQ (port 8089)
    ├──→ 👁️  Hand2DVisualizer (optional)
    └──→ 🧠 OrcaOperator
         ├─ Geometric Retargeting (21→17 joints)
         └─ OrcaHand.move_robot()
             ↓ ROS 2 (/orca_hand/command)
🤖 ORCA Hand Hardware (17 motors)
```

---

## 🔢 System Constants

| What | Value | Why |
|------|-------|-----|
| **Quest Keypoints** | 21 | MediaPipe hand model |
| **ORCA Joints** | 17 | 4 thumb + 3×4 fingers + 1 wrist |
| **Control Frequency** | 90 Hz | Matches Quest tracking rate |
| **Loop Period** | 11.1 ms | 1/90 Hz |
| **Wrist Range** | -50° to +30° | Extension to flexion |
| **Network Ports** | 8087-8089, 15001 | ZMQ communication |
| **End-to-End Latency** | ~30-50ms | Quest → ORCA movement |
| **Serial Baudrate** | 3000000 | ORCA hand USB (3 Mbaud) |

---

## 📡 Complete Data Flow (8 Layers)

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 1: VR HAND TRACKING (Quest Headset)                                    │
└──────────────────────────────────────────────────────────────────────────────┘

    👋 Human Hand
         │
         │ (MediaPipe ML - on-device)
         ↓
    Quest 3 Hand Tracking
         │ - 21 landmarks per hand
         │ - 90 Hz update rate
         │ - World coordinates (x,y,z in meters)
         │ - MediaPipe model: palm + 4 fingers × 5 joints
         ↓
    Unity/Quest App
         │ - Runs on headset (Android)
         │ - ZMQ Publisher
         │ - Sends hand_coords topic
         │
         │ ZMQ Socket: tcp://<YOUR_PC_IP>:8087
         ↓

┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 2: NETWORK TRANSPORT (ZMQ over WiFi)                                   │
└──────────────────────────────────────────────────────────────────────────────┘

    Network Interface
         │ host_address: <YOUR_PC_IP> (from configs/network.yaml)
         │ Protocol: ZeroMQ (pub/sub)
         │ Data: 21×3 float array (63 values)
         │ Format: JSON serialized
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
    │ - ZMQKeypointSubscriber(port 8087)                 │
    │ - Validates data                                   │
    │ - ZMQKeypointPublisher(port 8088)                  │
    │ - Republishes for OpenTeach ecosystem              │
    └─────────────────────────────────────────────────────┘
         │
         │ ZMQ topic: 'hand_coords'
         │ Port: 8088 (localhost)
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
         │ Port: 8089 (localhost)
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
    │ __init__():                                                             │
    │  - ZMQKeypointSubscriber(port 8089) → Quest data                       │
    │  - OrcaHand() → Robot wrapper                                          │
    │  - FrequencyTimer(90 Hz) → Timing                                      │
    │  - Config: handedness, thumb_dip_scale, gains, bias                    │
    │                                                                         │
    │ stream() loop (inherited from Operator base):                          │
    │  while True:                                                            │
    │      # Safety check                                                     │
    │      if robot.get_joint_position() is not None:                        │
    │          timer.start_loop()                                            │
    │          _apply_retargeted_angles()  # Core retargeting                │
    │          timer.end_loop()  # Sleep to maintain 90 Hz                   │
    │                                                                         │
    │ _apply_retargeted_angles():                                            │
    │  1. hand_keypoints = subscriber.recv_keypoints()  # 21×3 array         │
    │  2. joint_angles = quest21_to_orca17(...)  # Geometric retargeting     │
    │  3. robot.move_robot(joint_angles)  # Send to ORCA                     │
    └─────────────────────────────────────────────────────────────────────────┘
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
    │ __init__():                                                         │
    │  - Load ORCA config YAML (joint_ids, joint_roms, neutral)          │
    │  - JOINT_NAMES = ('thumb_mcp', 'thumb_abd', ..., 'wrist')          │
    │  - JOINT_LIMITS = np.array([[min, max], ...]) in radians           │
    │  - _control = DexArmControl() → ROS bridge                         │
    │  - _home_pose = neutral_position from config (in radians)          │
    │                                                                     │
    │ move_robot(joint_angles):  ← Called at 90 Hz                       │
    │  1. q_safe = _clamp(joint_angles)  # Enforce limits                │
    │  2. _control.send_joint_cmd(q_safe)  # To ROS bridge               │
    │  3. _last_cmd = q_safe  # Store for recording                      │
    │                                                                     │
    │ get_joint_position():  ← Safety check before each loop             │
    │  return _control.get_joint_angles()  # Returns None if disconnected│
    │                                                                     │
    │ Other methods:                                                      │
    │  - get_joint_state() → {'position': angles}                        │
    │  - get_commanded_joint_state() → {'position': _last_cmd}           │
    │  - home() → move to neutral position                               │
    │  - recorder_functions → dict for data collection                   │
    └─────────────────────────────────────────────────────────────────────┘
         │
         │ Commands via ROS bridge
         ↓

┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 7: ROS 2 BRIDGE (Communication Layer)                                  │
└──────────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────────────┐
    │ DexArmControl (ROS 2 Node)                                          │
    │ File: openteach/ros_links/orca_control.py                           │
    │                                                                     │
    │ ROS 2 Topics:                                                       │
    │  - SUBSCRIBE: /orca_hand/joint_states (sensor_msgs/JointState)     │
    │  - PUBLISH:   /orca_hand/command (std_msgs/Float64MultiArray)      │
    │                                                                     │
    │ __init__():                                                         │
    │  - rclpy.Node('orca_hand_link')                                    │
    │  - _latest_state = np.zeros(15)  # Thread-safe state               │
    │  - _lock = threading.Lock()                                        │
    │  - Publisher: /orca_hand/command                                   │
    │  - Subscriber: /orca_hand/joint_states                             │
    │                                                                     │
    │ send_joint_cmd(q):  ← 90 Hz from operator                          │
    │  msg = Float64MultiArray(data=q.tolist())                          │
    │  self.pub.publish(msg)                                             │
    │  → Publishes to /orca_hand/command topic                           │
    │                                                                     │
    │ get_joint_angles():  ← Safety check                                │
    │  with self._lock:                                                  │
    │      return self._latest_state.copy()                              │
    │                                                                     │
    │ _state_callback(msg):  ← ROS subscription                          │
    │  with self._lock:                                                  │
    │      self._latest_state[:] = msg.position                          │
    └─────────────────────────────────────────────────────────────────────┘
         │
         │ ROS 2 DDS Middleware (shared memory on same PC)
         ↓

┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 8: HARDWARE INTERFACE (ROS 2 Hardware Node)                            │
└──────────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────────────┐
    │ orca_hardware_node (ROS 2 Node)                                     │
    │ File: ros2_ws/src/orca_hardware_interface/...                       │
    │       orca_hardware_node.py                                         │
    │                                                                     │
    │ Subscribes: /orca_hand/command (Float64MultiArray)                 │
    │  → Receives 17 joint angles in radians                             │
    │  → Converts radians → degrees                                      │
    │  → Sends to orca_core                                              │
    │                                                                     │
    │ Uses: orca_core.OrcaHand                                           │
    │  hand = OrcaHand()                                                 │
    │  hand.connect()                                                    │
    │  hand.set_joint_pos(joint_dict_in_degrees)                         │
    │                                                                     │
    │ Publishes: /orca_hand/joint_states (JointState)                    │
    │  → Reads current positions from hardware                           │
    │  → Converts degrees → radians                                      │
    │  → Publishes at ~90 Hz                                             │
    │                                                                     │
    │ Serial Communication:                                              │
    │  Port: <YOUR_SERIAL_PORT> (e.g., /dev/tty.usbserial-XXX)          │
    │  Baudrate: 3000000 (3 Mbaud)                                       │
    │  Protocol: Dynamixel Protocol 2.0                                  │
    │                                                                     │
    │ Motor Commands (17 DOF):                                           │
    │  - thumb_mcp, thumb_abd, thumb_pip, thumb_dip                      │
    │  - index_abd, index_mcp, index_pip                                 │
    │  - middle_abd, middle_mcp, middle_pip                              │
    │  - ring_abd, ring_mcp, ring_pip                                    │
    │  - pinky_abd, pinky_mcp, pinky_pip                                 │
    │  - wrist (flexion/extension)                                       │
    └─────────────────────────────────────────────────────────────────────┘
         │
         │ Serial/USB (Dynamixel Protocol 2.0)
         ↓
    🤖 Physical ORCA Hand Moves!

```

---

## 🎯 Geometric Retargeting (The Brain)

The core algorithm that converts Quest hand pose → ORCA joint angles.

### Algorithm: `quest21_to_orca17()`

```python
# INPUT: Quest hand keypoints (21 points in 3D world space)
kp_world = [[x, y, z], ...]  # 21 × 3 numpy array

# STEP 1: Build Palm Frame (rotation-invariant coordinate system)
wrist = kp_world[0]
index_mcp = kp_world[5]
pinky_mcp = kp_world[17]
middle_mcp = kp_world[9]

xhat = normalize(index_mcp - pinky_mcp)  # Across palm (left-right)
yhat = normalize(middle_mcp - wrist)      # To fingers (forward)
zhat = cross(xhat, yhat)                  # Palm normal (up)

# Rotation matrix from world to palm frame
R = [xhat, yhat, zhat]

# Transform all points to palm-centered, palm-oriented frame
kp_palm = (kp_world - wrist) @ R.T

# STEP 2: Compute Joint Angles Geometrically
angles = {}

# For each finger (thumb, index, middle, ring, pinky):
#
# FLEXION (MCP, PIP, DIP):
#   - Get bone vectors (e.g., mcp→pip, pip→dip)
#   - Interior angle = arccos(dot(v1, v2))
#   - Always positive (0 = straight, π/2 = curled)
#
# ABDUCTION:
#   - Project finger base onto palm plane (remove z component)
#   - Signed angle from xhat (palm width axis)
#   - Positive = spread away from center
#   - Negative = squeeze toward center
#   - For left hand: negate (mirror symmetry)
#
# WRIST (if enable_wrist=true):
#   - Measure palm yhat tilt relative to gravity [-1,0,0]
#   - wrist = arcsin(dot(yhat, -gravity))
#   - Positive = flexion (palm down)
#   - Negative = extension (palm up)

# STEP 3: Apply Calibration (per-joint tuning)
for joint_name in orca_joint_order:
    angle = angles_dict[joint_name]
    
    # Gain (scale factor)
    if joint_name in joint_gain:
        angle *= joint_gain[joint_name]
    
    # Bias (offset)
    if joint_name in joint_bias:
        angle += joint_bias[joint_name]
    
    output.append(angle)

# OUTPUT: 17 joint angles in radians
# Order: [thumb_mcp, thumb_abd, thumb_pip, thumb_dip,
#         index_abd, index_mcp, index_pip,
#         middle_abd, middle_mcp, middle_pip,
#         ring_abd, ring_mcp, ring_pip,
#         pinky_abd, pinky_mcp, pinky_pip,
#         wrist]
```

### Why Palm Frame?

**Problem:** Quest hand can be at any orientation in space
**Solution:** Define intrinsic coordinate system from palm geometry

**Benefits:**
- **Rotation-invariant:** Works regardless of hand orientation
- **Consistent:** Same hand pose → same angles
- **Intuitive:** x=width, y=forward, z=up relative to palm

### Flexion vs Abduction

| Type | Meaning | Computation | Sign |
|------|---------|-------------|------|
| **Flexion** | Finger curl | `arccos(dot(bone1, bone2))` | Always positive |
| **Abduction** | Finger spread | `atan2(projection, reference)` | Signed (±) |

**Left Hand Mirror:**
- Flexion: Same direction for both hands ✓
- Abduction: Negated for left hand (mirror) ✗
- Wrist: Same direction (flex down, extend up) ✓

---

## 🗂️ File Structure

```
OpenTeach_Orca/
│
├── configs/
│   ├── network.yaml              # Network ports and IPs
│   ├── teleop.yaml              # Main config (selects robot)
│   └── robot/
│       └── orca.yaml            # ORCA-specific config (Hydra)
│
├── openteach/
│   ├── components/
│   │   ├── detector/
│   │   │   ├── oculus.py        # Layer 3: OculusVRHandDetector
│   │   │   └── keypoint_transform.py  # Layer 4: Transform
│   │   │
│   │   ├── operators/
│   │   │   ├── operator.py      # Base class (stream loop)
│   │   │   └── orca_operator.py # Layer 5: OrcaOperator
│   │   │
│   │   └── visualizers/
│   │       └── visualizer_2d.py # Layer 5a: Visualizer
│   │
│   ├── robot/
│   │   ├── robot.py             # RobotWrapper base class
│   │   └── orca.py              # Layer 6: OrcaHand wrapper
│   │
│   ├── ros_links/
│   │   └── orca_control.py      # Layer 7: ROS bridge
│   │
│   └── utils/
│       ├── network.py           # ZMQ helpers
│       └── timer.py             # FrequencyTimer (90 Hz)
│
├── ros2_ws/
│   └── src/
│       └── orca_hardware_interface/
│           └── orca_hardware_interface/
│               ├── orca_hardware_node.py  # Layer 8: Hardware
│               └── __init__.py
│
├── orca_core/                   # Submodule (orca_core library)
│   └── orca_core/
│       ├── core.py              # OrcaHand class (motor control)
│       ├── hardware/
│       │   └── dynamixel_client.py  # Dynamixel Protocol 2.0
│       └── models/
│           └── orcahand_v1_right/
│               └── config.yaml  # Joint IDs, limits, neutral
│
└── teleop.py                    # Main entry point (Hydra)
```

---

## ⚙️ Configuration Hierarchy (Hydra)

```yaml
# Entry point: python teleop.py robot=orca
#
# Loads: configs/teleop.yaml
defaults:
  - robot: orca  # ← Selects configs/robot/orca.yaml

# configs/robot/orca.yaml
detector:
  _target_: openteach.components.detector.oculus.OculusVRHandDetector
  config:
    host_address: ${network.host_address}  # From network.yaml
    port: ${network.keypoint_port}

transforms:
  - _target_: ...TransformHandPositionCoords
    config: {...}

operators:
  - _target_: openteach.components.operators.orca_operator.OrcaOperator
    config:
      handedness: right           # or "left"
      thumb_dip_scale: 0.3        # DIP coupling factor
      enable_wrist: true          # Use gravity-based wrist
      joint_gain: {}              # Per-joint multipliers
      joint_bias: {}              # Per-joint offsets

controllers:
  - _target_: openteach.robot.orca.OrcaHand

# configs/network.yaml
host_address: "<YOUR_PC_IP>"      # Your PC's WiFi IP
oculus_ip: "<YOUR_QUEST_IP>"      # Quest's WiFi IP
keypoint_port: 8087              # Quest → PC
zmq_detector_pub_port: 8088      # Detector → Transform
zmq_transform_pub_port: 8089     # Transform → Operator
zmq_vis_pub_port: 15001          # Visualizer → Quest

# orca_core/models/orcahand_v1_right/config.yaml
joint_ids: [thumb_mcp, thumb_abd, thumb_pip, thumb_dip,
            index_abd, index_mcp, index_pip,
            ...]
joint_roms:  # Range of motion in degrees
  wrist: [-50, 30]
  thumb_mcp: [0, 90]
  index_mcp: [0, 90]
  ...
neutral_position:  # Home position in degrees
  wrist: 0
  thumb_mcp: 10
  ...
port: "<YOUR_SERIAL_PORT>"       # e.g., /dev/ttyUSB0
baudrate: 3000000
```

---

## 🔄 Timing & Frequencies

| Component | Frequency | Period | Why |
|-----------|-----------|--------|-----|
| Quest Hand Tracking | 90 Hz | 11.1 ms | Native Quest 3 rate |
| OculusVRHandDetector | 90 Hz | 11.1 ms | Matches Quest output |
| TransformHandPositionCoords | 90 Hz | 11.1 ms | Pass-through + filter |
| OrcaOperator Loop | 90 Hz | 11.1 ms | VR_FREQ constant |
| Hand2DVisualizer | 90 Hz | 11.1 ms | Real-time feedback |
| /orca_hand/command | 90 Hz | 11.1 ms | Published by ROS bridge |
| /orca_hand/joint_states | ~90 Hz | ~11 ms | Published by hardware node |
| Motor update | ~90 Hz | ~11 ms | Dynamixel sync write |

**Critical:** `FrequencyTimer` enforces 90 Hz by sleeping remainder of 11.1ms period after processing.

---

## 🔒 Safety Mechanisms

### 1. Joint Limit Clamping (Software)
```python
# In openteach/robot/orca.py
def _clamp(self, q: np.ndarray) -> np.ndarray:
    q_min, q_max = self.JOINT_LIMITS.T
    return np.clip(q, q_min, q_max)
```
- Enforces limits before sending to ROS
- Limits loaded from ORCA-core config YAML
- Example: wrist [-50°, +30°], fingers [0°, 90°]

### 2. Robot Alive Check
```python
# In openteach/components/operators/operator.py
while True:
    if self.robot.get_joint_position() is not None:
        # Robot is connected and responding
        self._apply_retargeted_angles()
    else:
        # Skip this loop - robot disconnected
        continue
```
- Checks every loop iteration
- Returns `None` if no ROS messages received
- Prevents commanding disconnected robot

### 3. Thread-Safe ROS Bridge
```python
# In openteach/ros_links/orca_control.py
self._lock = threading.Lock()

def get_joint_angles(self):
    with self._lock:
        return self._latest_state.copy()
```
- Prevents race conditions
- ROS callback updates state in background thread
- Main thread reads safely

### 4. Hardware-Level Limits
- ORCA firmware enforces joint limits
- Current limiting on motors
- Emergency stop handling
- Torque limiting to prevent damage

---

## 📊 Data Types at Each Stage

| Stage | Type | Shape | Example |
|-------|------|-------|---------|
| Quest → ZMQ | JSON (float list) | 21×3 | `[[0.0, 0.5, 0.1], ...]` |
| Detector output | `np.ndarray` | (21, 3) | NumPy float64 |
| Transform output | `np.ndarray` | (21, 3) | Smoothed coords |
| Retargeting output | `np.ndarray` | (17,) | `[0.5, 0.3, 0.8, ...]` rad |
| ROS command | `Float64MultiArray` | 17 | `{data: [0.5, 0.3, ...]}` |
| ROS feedback | `JointState` | 17 | `{position: [...], velocity: [...]}` |
| orca_core input | `dict` | 17 | `{'thumb_mcp': 5.0, ...}` deg |
| Dynamixel command | `int` | 17 | `[1587, 3298, ...]` units |

---

## 🎮 Operation Modes

### Teleoperation Mode (Active Control)
```bash
python teleop.py robot=orca
```
- **Purpose:** Real-time hand control
- **Frequency:** 90 Hz
- **Methods Used:** `get_joint_position()`, `move_robot()`
- **Recording:** No data saved

### Data Collection Mode
```bash
python data_collect.py robot=orca
```
- **Purpose:** Record demonstrations for learning
- **Frequency:** 90 Hz recording
- **Methods Used:** `recorder_functions`, `get_joint_state()`, `get_commanded_joint_state()`
- **Output:** `demonstration_N/` folder with joint trajectories

### Visualization Mode
```yaml
# In configs/teleop.yaml
visualize_right_2d: true
```
- **Purpose:** Debug hand tracking visually
- **Runs:** Hand2DVisualizer in parallel process
- **Shows:** Matplotlib skeleton plot of hand
- **Streams:** Back to Quest at port 15001

---

## 🧩 Design Patterns

### 1. Multiprocessing Architecture
- Each component = separate Python process
- Decoupled via ZMQ pub/sub
- Can run on different machines
- Fault isolation (one crash doesn't kill all)

### 2. Hydra Dependency Injection
- Components instantiated from `_target_` config paths
- No hardcoded imports in main script
- Easy robot swapping: `robot=orca` → `robot=franka`
- Configuration composition with defaults

### 3. Abstract Base Classes
- `Operator`: Forces `_apply_retargeted_angles()` implementation
- `RobotWrapper`: Enforces standard interface
- Enables polymorphism (same code, different robots)

### 4. Geometric (Non-ML) Retargeting
- Pure NumPy math (fast)
- Rotation-invariant (palm-frame based)
- No training data required
- Deterministic and debuggable

### 5. Configuration Hierarchy
```
Main:     teleop.yaml    (selects robot)
Network:  network.yaml   (ports, IPs)
Robot:    robot/orca.yaml (components, params)
Hardware: orca_core/.../config.yaml (joint specs)
```

---

## 🐛 Debugging Commands

### Check Quest Connection
```bash
# Test if Quest is publishing hand data
python -c "
from openteach.utils.network import ZMQKeypointSubscriber
s = ZMQKeypointSubscriber('<YOUR_PC_IP>', 8087, 'hand_coords')
print('Received keypoints:', s.recv_keypoints().shape)
"
# Expected: Received keypoints: (21, 3)
```

### Check ROS Topics
```bash
# List all ORCA topics
ros2 topic list | grep orca_hand

# Monitor commands being sent
ros2 topic echo /orca_hand/command

# Monitor joint states (feedback)
ros2 topic echo /orca_hand/joint_states
```

### Test Retargeting
```python
# Test geometric conversion in isolation
from openteach.components.operators.orca_operator import quest21_to_orca17
import numpy as np

# Fake Quest data
keypoints = np.random.rand(21, 3)

# Convert to ORCA angles
angles = quest21_to_orca17(
    kp_world=keypoints,
    orca_joint_order=("thumb_mcp", "thumb_abd", ..., "wrist"),
    handedness="right"
)

print(f"Output shape: {angles.shape}")  # Should be (17,)
print(f"Wrist angle: {angles[-1]} rad")  # Last element
```

### Verify Joint Names
```python
# Check ORCA joint order matches retargeting
from openteach.robot.orca import OrcaHand
hand = OrcaHand()
print("Joint names:", hand.JOINT_NAMES)
print("Joint limits:", hand.JOINT_LIMITS)
```

---

## 📈 Performance Metrics

### Expected Latencies
| Component | Latency | Notes |
|-----------|---------|-------|
| Quest tracking | <5 ms | Native MediaPipe on-device |
| WiFi transport | 10-20 ms | Depends on network quality |
| ZMQ hop | ~1 ms | Localhost, shared memory |
| Retargeting math | <1 ms | Pure NumPy on modern CPU |
| ROS publish | ~1 ms | DDS shared memory |
| Serial → Motors | ~2 ms | Dynamixel Protocol 2.0 |
| **Total end-to-end** | **~30-50 ms** | Quest gesture → ORCA motion |

### At 90 Hz
- **Loop period:** 11.1 ms
- **Compute budget:** ~10 ms (1 ms margin for jitter)
- **If loop takes >11ms:** Warning printed, control degrades

---

## 🔧 Common Commands

```bash
# Run teleoperation
python teleop.py robot=orca

# Run with left hand
# (Edit configs/robot/orca.yaml: handedness: left)
python teleop.py robot=orca

# Collect demonstration data
python data_collect.py robot=orca

# Launch ROS 2 hardware node (separate terminal)
cd <YOUR_REPO_PATH>/OpenTeach_Orca/ros2_ws
source install/setup.bash
ros2 launch orca_hardware_interface orca_hardware.launch.py

# Enable visualization (edit configs/teleop.yaml first)
visualize_right_2d: true
python teleop.py robot=orca
```

---

## 🎓 Key Concepts

### Palm Frame
- **Why:** Makes retargeting work at any hand orientation
- **How:** Define coordinate system from palm geometry
- **Benefit:** Rotation-invariant angles

### Flexion (Curl)
- **Always positive angle** (0 = straight, π/2 = fully curled)
- **Computation:** `arccos(dot(bone1, bone2))`
- **Same for both hands**

### Abduction (Spread)
- **Signed angle** (+ = spread out, - = squeeze in)
- **Computation:** `atan2(projection, reference_axis)`
- **Mirrored for left hand** (negated)

### Wrist Estimation
- **Challenge:** Quest has no wrist joint tracking
- **Solution:** Estimate from palm tilt vs gravity
- **Formula:** `arcsin(dot(palm_forward, -gravity))`
- **Range:** -50° (extension/up) to +30° (flexion/down)

### Left Hand Mirroring
| Joint Type | Left vs Right |
|------------|---------------|
| Flexion | Same direction |
| Abduction | Negated (mirror) |
| Wrist | Same direction |

---

## 📚 Related Documentation

- **GETTING_STARTED.md** - Step-by-step setup tutorial
- **add_your_own_robot.md** - OpenTeach integration guide
- **teleop_data_collect.md** - Data collection guide
- **vr.md** - Quest setup instructions
- **ros2_ws/README.md** - ROS 2 workspace setup

---

**Complete system architecture for ORCA Hand teleoperation** 🤖✋
