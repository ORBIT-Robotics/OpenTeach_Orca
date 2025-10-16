# ORCA Hardware Interface - ROS 2 Workspace

This ROS 2 workspace provides the **hardware driver node** that bridges OpenTeach commands to the physical ORCA Hand hardware.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    COMPLETE SYSTEM                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  OpenTeach (Python process)                                │
│      ├── orca_operator.py       (90 Hz control loop)       │
│      ├── orca_control.py        (ROS 2 Python client)      │
│      └── Publishes: /orca_hand/command (Float64MultiArray) │
│                                                             │
│                     ↓↑ ROS 2 Topics ↓↑                     │
│                                                             │
│  ROS 2 Workspace (THIS FOLDER)                             │
│      ├── orca_hardware_node     (Hardware driver)          │
│      ├── Subscribes: /orca_hand/command                    │
│      ├── Publishes: /orca_hand/joint_states (JointState)   │
│      └── Uses: orca_core.OrcaHand                          │
│                                                             │
│                     ↓↑ Serial (pyserial) ↓↑                │
│                                                             │
│  ORCA Hand Hardware                                         │
│      └── 17 Dynamixel motors on hand                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Package: `orca_hardware_interface`

**Purpose:** ROS 2 node that communicates with ORCA hand hardware using `orca_core` library.

**Key Features:**
- Subscribes to `/orca_hand/command` topic for joint commands (radians)
- Publishes `/orca_hand/joint_states` topic for feedback (radians)
- Uses `orca_core.OrcaHand` for direct hardware communication
- Runs at 90 Hz to match OpenTeach control rate
- Thread-safe command handling
- Auto-connect and calibration options

## Prerequisites

### 1. Install ROS 2 Humble

**macOS Installation:**
```bash
# Install ROS 2 Humble via Homebrew (if available) or from source
# See: https://docs.ros.org/en/humble/Installation.html

# Or use Docker (recommended for macOS):
docker pull ros:humble
```

**Ubuntu Installation:**
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
```

### 2. Install Python Dependencies

**CRITICAL:** orca_core MUST be installed in your Python environment!

```bash
# Install orca_core (from submodule in OpenTeach_Orca)
# Replace <YOUR_REPO_PATH> with where you cloned OpenTeach_Orca

# Linux/macOS:
cd <YOUR_REPO_PATH>/OpenTeach_Orca/orca_core
# Example: cd ~/projects/OpenTeach_Orca/orca_core

# Windows:
cd <YOUR_REPO_PATH>\OpenTeach_Orca\orca_core
# Example: cd C:\Users\YourName\OpenTeach_Orca\orca_core

pip install -e .

# Verify installation
python -c "from orca_core import OrcaHand; print('✓ orca_core installed')"

# Install ROS 2 Python dependencies
pip install rclpy sensor-msgs std-msgs
```

**Why pip install?**
- Makes `orca_core` importable from anywhere
- No hardcoded paths needed
- Works on any PC after installation
- Proper Python package management

### 3. Source ROS 2 Environment

```bash
# Ubuntu
source /opt/ros/humble/setup.bash

# macOS (if installed locally)
source /path/to/ros2_humble/setup.bash
```

## Build Instructions

### 1. Build the workspace

```bash
# Navigate to the ros2_ws directory
# Replace <YOUR_REPO_PATH> with where you cloned OpenTeach_Orca

# Linux/macOS:
cd <YOUR_REPO_PATH>/OpenTeach_Orca/ros2_ws
# Example: cd ~/projects/OpenTeach_Orca/ros2_ws

# Windows (WSL/Ubuntu):
cd <YOUR_REPO_PATH>/OpenTeach_Orca/ros2_ws
# Example: cd /mnt/c/Users/YourName/OpenTeach_Orca/ros2_ws

# Build with colcon
colcon build

# Source the workspace
source install/setup.bash
```

### 2. Verify build

```bash
# Check if package is visible
ros2 pkg list | grep orca

# Should show:
# orca_hardware_interface
```

## Usage

### Basic Usage (Auto-connect)

```bash
# Terminal 1: Launch hardware node
# Replace <YOUR_REPO_PATH> with where you cloned OpenTeach_Orca

# Linux/macOS:
cd <YOUR_REPO_PATH>/OpenTeach_Orca/ros2_ws
source install/setup.bash
ros2 launch orca_hardware_interface orca_hardware.launch.py

# Terminal 2: Run OpenTeach teleoperation
cd <YOUR_REPO_PATH>/OpenTeach_Orca
python teleop.py robot=orca

# Windows (WSL/Ubuntu): Same commands, just use your Windows path
# Example: cd /mnt/c/Users/YourName/OpenTeach_Orca/ros2_ws
```

### With Custom Model Path

```bash
ros2 launch orca_hardware_interface orca_hardware.launch.py \
    model_path:=/path/to/orca_core/models/orcahand_v1_right
```

### With Calibration

```bash
# WARNING: Calibration takes several minutes!
ros2 launch orca_hardware_interface orca_hardware.launch.py \
    auto_calibrate:=true
```

### Manual Node Execution

```bash
# Run node directly (without launch file)
ros2 run orca_hardware_interface orca_hardware_node --ros-args \
    -p model_path:="" \
    -p control_frequency:=90.0 \
    -p auto_connect:=true \
    -p auto_calibrate:=false
```

## Testing

### 1. Check Topics

```bash
# List all topics
ros2 topic list

# Should show:
# /orca_hand/command
# /orca_hand/joint_states

# Monitor command stream
ros2 topic echo /orca_hand/command

# Monitor joint states
ros2 topic echo /orca_hand/joint_states
```

### 2. Send Test Command

```bash
# Send a test command (all zeros)
ros2 topic pub /orca_hand/command std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" \
    --once
```

### 3. Check Node Status

```bash
# List running nodes
ros2 node list

# Should show:
# /orca_hardware_node

# Get node info
ros2 node info /orca_hardware_node
```

## Troubleshooting

### "Could not import orca_core"
```bash
# Make sure orca_core is installed
cd ../orca_core
pip install -e .
```

### "Connection failed" on startup
- Check USB connection to ORCA hand
- Verify serial port permissions: `ls -l /dev/tty*`
- On Linux: `sudo usermod -a -G dialout $USER` (then reboot)
- Check orca_core config.yaml has correct port

### "No topics visible"
```bash
# Make sure ROS 2 environment is sourced
source /opt/ros/humble/setup.bash  # Ubuntu
source install/setup.bash          # This workspace

# Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID
```

### Build errors
```bash
# Clean build
rm -rf build install log
colcon build --symlink-install
```

## Configuration

### Hardware Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | `""` (auto) | Path to ORCA model directory |
| `control_frequency` | `90.0` | Control loop rate (Hz) |
| `auto_connect` | `true` | Connect to hardware on startup |
| `auto_calibrate` | `false` | Run calibration on startup |

### Modifying Parameters

Edit launch file: `src/orca_hardware_interface/launch/orca_hardware.launch.py`

## Development

### Code Structure

```
ros2_ws/
├── src/
│   └── orca_hardware_interface/
│       ├── orca_hardware_interface/
│       │   ├── __init__.py
│       │   └── orca_hardware_node.py    ← Main hardware driver
│       ├── launch/
│       │   └── orca_hardware.launch.py  ← Launch configuration
│       ├── resource/
│       ├── package.xml                   ← ROS 2 package manifest
│       └── setup.py                      ← Python package setup
├── build/                                ← Generated (gitignored)
├── install/                              ← Generated (gitignored)
└── log/                                  ← Generated (gitignored)
```

### Adding New Features

To add features to the hardware node:
1. Edit `orca_hardware_interface/orca_hardware_node.py`
2. Rebuild: `colcon build`
3. Re-source: `source install/setup.bash`
4. Test changes

## Safety Notes

⚠️ **IMPORTANT:**
- Always have emergency stop ready
- Keep hands clear when motors are enabled
- Calibration moves ALL motors - ensure hand is free to move
- Monitor motor currents during operation
- Use `auto_calibrate:=false` until you verify hardware setup

## Integration with OpenTeach

This node is **automatically used** by OpenTeach when you run:
```bash
python teleop.py robot=orca
```

The integration works as follows:
1. OpenTeach's `orca_control.py` creates a ROS 2 client
2. This node must be running before OpenTeach starts
3. Commands flow: Quest 3 → OpenTeach → ROS topics → This node → Hardware
4. Feedback flows: Hardware → This node → ROS topics → OpenTeach

## License

MIT License (same as OpenTeach and orca_core)

## Support

For issues specific to:
- **Hardware node**: Check this README and node logs
- **orca_core library**: See orca_core documentation
- **OpenTeach integration**: See main OpenTeach docs
- **ROS 2 general**: See ROS 2 documentation
