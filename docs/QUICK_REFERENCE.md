# ORCA Hand Teleoperation - Quick Reference

One-page visual summary of the system.

---

## 📊 System at a Glance

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

## 🔢 The Magic Numbers

| What | Value | Why |
|------|-------|-----|
| **Quest Keypoints** | 21 | MediaPipe hand model |
| **ORCA Joints** | 17 | 4 thumb + 3×4 fingers + 1 wrist |
| **Control Frequency** | 90 Hz | Matches Quest tracking rate |
| **Wrist Range** | -50° to +30° | Extension to flexion |
| **Network Ports** | 8087-8089, 15001 | ZMQ communication |
| **End-to-End Latency** | ~30-50ms | Quest → ORCA movement |

---

## 📁 3 Critical Files You Created

### 1️⃣ **Robot Wrapper** (`openteach/robot/orca.py`)
```python
class OrcaHand(RobotWrapper):
    def move(self, angles):      # Send commands (90 Hz)
    def get_joint_position():    # Safety check
    def home():                  # Reset to neutral
```
**Role:** Interface between OpenTeach and ORCA hardware

---

### 2️⃣ **Operator** (`openteach/components/operators/orca_operator.py`)
```python
class OrcaOperator(Operator):
    def _apply_retargeted_angles(self):
        keypoints = subscriber.recv_keypoints()  # Quest → 21 points
        angles = quest21_to_orca17(keypoints)     # Geometry → 17 joints
        robot.move_robot(angles)                  # Command → ORCA
```
**Role:** Convert Quest hand pose to ORCA joint angles

---

### 3️⃣ **ROS Bridge** (`openteach/ros_links/orca_control.py`)
```python
class DexArmControl(Node):
    def send_joint_cmd(q):      # Publish to /orca_hand/command
    def get_joint_angles():     # Read from /orca_hand/joint_states
```
**Role:** ROS 2 communication layer

---

## 🎯 Retargeting Pipeline (The Brain)

```python
# INPUT: Quest hand (21 keypoints × 3D coords)
kp_world = [[x, y, z], ...]  # 21 × 3 array

# STEP 1: Build rotation-invariant palm frame
xhat = index_mcp → pinky_mcp  (across palm)
yhat = wrist → middle_mcp      (to fingers)
zhat = xhat × yhat             (palm normal)

# STEP 2: Transform all points to palm frame
kp_palm = (kp_world - wrist) @ [xhat, yhat, zhat]

# STEP 3: Compute angles geometrically
For each finger:
    flexion = arccos(dot(proximal, distal))      # Interior angle
    abduction = atan2(projection onto palm plane) # Signed angle

wrist = arcsin(palm_tilt vs gravity)

# STEP 4: Apply calibration
angle = gain * angle + bias

# OUTPUT: 17 joint angles (radians)
[thumb_mcp, thumb_abd, thumb_pip, thumb_dip,
 index_abd, index_mcp, index_pip,
 middle_abd, middle_mcp, middle_pip,
 ring_abd, ring_mcp, ring_pip,
 pinky_abd, pinky_mcp, pinky_pip,
 wrist]
```

---

## ⚡ Active Teleoperation (90 Hz Loop)

```python
while True:  # Main control loop
    
    # 1. SAFETY CHECK (read actual robot state)
    if robot.get_joint_position() is None:
        continue  # Robot disconnected, skip this iteration
    
    # 2. START TIMER
    timer.start_loop()
    
    # 3. RECEIVE hand pose from Quest
    keypoints = zmq_subscriber.recv_keypoints()
    
    # 4. RETARGET to robot space
    joint_angles = quest21_to_orca17(keypoints)
    
    # 5. SEND command to robot
    robot.move_robot(joint_angles)  # → ROS topic → Hardware
    
    # 6. SLEEP to maintain 90 Hz
    timer.end_loop()  # Sleeps if loop was faster than 11ms
```

---

## 🗺️ Configuration Hierarchy

```yaml
# configs/teleop.yaml (main entry point)
defaults:
  - robot: orca        # ← Selects which robot config to load

# configs/network.yaml (network ports)
host_address: 172.24.71.206
keypoint_port: 8088
transformed_position_keypoint_port: 8089

# configs/robot/orca.yaml (robot-specific)
operators:
  - _target_: openteach.components.operators.orca_operator.OrcaOperator
    config:
      handedness: right           # or "left"
      thumb_dip_scale: 0.3        # DIP coupling
      enable_wrist: true          # gravity-based wrist
      joint_gain: {}              # per-joint multipliers
      joint_bias: {}              # per-joint offsets

# orca_core/models/orcahand_v1_right/config.yaml (hardware spec)
joint_ids: [thumb_mcp, thumb_abd, ...]
joint_roms:                      # ROM in degrees
  wrist: [-50, 30]
  thumb_mcp: [0, 90]
  ...
```

---

## 🔧 Commands

### Run Teleoperation
```bash
python teleop.py robot=orca
```

### Enable Visualization
```bash
# Edit configs/teleop.yaml:
visualize_right_2d: true

python teleop.py robot=orca
```

### Switch to Left Hand
```bash
# Edit configs/robot/orca.yaml:
handedness: left

python teleop.py robot=orca
```

### Collect Data
```bash
python data_collect.py robot=orca
```

### Check ROS Topics
```bash
# See ORCA joint states
ros2 topic echo /orca_hand/joint_states

# See commands being sent
ros2 topic echo /orca_hand/command
```

---

## 🐛 Troubleshooting Checklist

### ❌ Quest not sending data
```bash
# Test ZMQ connection
python -c "from openteach.utils.network import ZMQKeypointSubscriber; \
           s = ZMQKeypointSubscriber('172.24.71.206', 8087, 'hand_coords'); \
           print('Received:', s.recv_keypoints().shape)"
```
**Expected:** `Received: (21, 3)`

### ❌ Robot not responding
```bash
# Check ROS 2 connection
ros2 topic list | grep orca_hand
```
**Expected:** `/orca_hand/command` and `/orca_hand/joint_states`

### ❌ Wrong joint order
```python
# Verify joint names match ORCA config
from openteach.robot.orca import OrcaHand
hand = OrcaHand()
print(hand.JOINT_NAMES)
```
**Expected:** `('thumb_mcp', 'thumb_abd', ..., 'wrist')`

### ❌ Angles out of range
```python
# Check joint limits
print(hand.JOINT_LIMITS)
```
**Expected:** 17×2 array with [min, max] in radians

---

## 📊 Data Flow Summary

| Layer | Input | Processing | Output |
|-------|-------|------------|--------|
| **Quest** | Hand pose | MediaPipe tracking | 21 keypoints @ 90Hz |
| **Detector** | Raw keypoints | Validate & republish | ZMQ topic @ 90Hz |
| **Transform** | Raw coords | Filter & align | Transformed coords @ 90Hz |
| **Operator** | Transformed coords | Geometric retargeting | 17 joint angles @ 90Hz |
| **Robot Wrapper** | Joint angles | Clamp to limits | Safe commands @ 90Hz |
| **ROS Bridge** | Safe commands | Serialize & publish | ROS topic @ 90Hz |
| **ORCA Hand** | ROS commands | Motor control | Physical movement |

---

## 🎓 Key Concepts

### Palm Frame
- **Why:** Makes retargeting rotation-invariant
- **How:** Define coordinate system from palm geometry
- **Benefit:** Works regardless of hand orientation in world space

### Flexion vs Abduction
- **Flexion:** Finger curl (always positive angle)
  - `angle = arccos(dot(bone1, bone2))`
- **Abduction:** Finger spread (signed, left/right)
  - `angle = atan2(projection, reference_axis)`

### Left Hand Mirroring
- **Flexion:** Same direction for both hands
- **Abduction:** Negated for left hand (mirror symmetry)
- **Wrist:** Same direction (both flex down, extend up)

### Wrist Estimation
- **Challenge:** Quest has no wrist joint tracking
- **Solution:** Estimate from palm tilt vs gravity
- **Formula:** `arcsin(dot(palm_yhat, -gravity))`
- **Range:** -50° (extension) to +30° (flexion)

---

## 🚀 Next Steps

1. **Test with Real Hardware**
   - Start ORCA ROS node
   - Run `python teleop.py robot=orca`
   - Move your hand in front of Quest

2. **Tune Sensitivity**
   ```yaml
   joint_gain:
     wrist: 0.8              # Reduce wrist movement
     index_mcp: 1.2          # Increase index curl
   ```

3. **Collect Demonstrations**
   ```bash
   python data_collect.py robot=orca
   # Perform task with Quest tracking
   # Data saved to demonstration_N/
   ```

4. **Train Policies**
   - Use recorded `joint_states.txt` and `commanded_joint_states.txt`
   - Train imitation learning model
   - Deploy with `deploy_server.py`

---

## 📚 Documentation Files

- `ORCA_SYSTEM_ARCHITECTURE.md` - Full technical details (this summary's parent)
- `add_your_own_robot.md` - OpenTeach integration checklist
- `teleop_data_collect.md` - Data collection guide
- `vr.md` - Quest setup instructions

---

**Made with ❤️ for ORCA Hand teleoperation** 🤖✋
