# Getting Started - ORCA Hand Teleoperation

Complete tutorial to get Meta Quest 3 controlling your ORCA Hand.

---

## 📋 Prerequisites

Before starting, ensure you have:

### Hardware
- ✅ **ORCA Hand** (right or left) with USB cable
- ✅ **Meta Quest 3** headset
- ✅ **PC/Laptop** (Linux/macOS/Windows with WSL)
- ✅ **WiFi Router** (Quest and PC must be on same network)

### Software (will install in this guide)
- Python 3.8+
- ROS 2 (Humble or Jazzy)
- Git with LFS
- orca_core library

---

## 🚀 Quick Start (5 Steps)

```
Step 1: Clone & Install      (15 min)
Step 2: Find Your Addresses  (5 min)
Step 3: Configure System     (5 min)
Step 4: Build ROS Workspace  (10 min)
Step 5: Run Teleoperation    (2 min)
```

---

## Step 1: Clone Repository & Install Dependencies

### 1.1 Clone OpenTeach_Orca

```bash
# Linux/macOS:
cd <YOUR_PROJECTS_FOLDER>
git clone --recursive https://github.com/ORBIT-Robotics/OpenTeach_Orca.git
cd OpenTeach_Orca

# Windows (WSL/Ubuntu) - Run in WSL terminal, NOT PowerShell!:
cd /mnt/c/Users/<YourName>/Projects
git clone --recursive https://github.com/ORBIT-Robotics/OpenTeach_Orca.git
cd OpenTeach_Orca
```

**Note:** `--recursive` is important! It pulls the `orca_core` submodule.

**⚠️ WSL Users:** All Python, ROS, and OpenTeach commands must run in your **WSL/Ubuntu terminal**, not Windows PowerShell! PowerShell is only for USB passthrough (`usbipd`) and firewall configuration.

### 1.2 Install Python Dependencies

```bash
# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate  # Linux/macOS
# Or: venv\Scripts\activate  # Windows CMD
# NOTE: WSL users should use the Linux/macOS command in WSL terminal!

# Install OpenTeach
pip install -e .

# Install orca_core (CRITICAL!)
cd orca_core
pip install -e .
cd ..

# Verify installation
python -c "from orca_core import OrcaHand; print('✓ orca_core installed')"
python -c "from openteach.robot.orca import OrcaHand; print('✓ OpenTeach ORCA installed')"
```

### 1.3 Install ROS 2 (Humble or Jazzy)

**Check if ROS 2 is already installed:**
```bash
which ros2
# If it shows /opt/ros/jazzy/bin/ros2 or /opt/ros/humble/bin/ros2, you're good!
```

**If not installed - Ubuntu/WSL:**
```bash
# Add ROS 2 apt repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Install ROS 2 Humble (or Jazzy)
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
# OR for Jazzy: sudo apt install ros-jazzy-desktop python3-colcon-common-extensions

# Source ROS 2 (replace 'humble' with 'jazzy' if you installed Jazzy)
source /opt/ros/humble/setup.bash
# OR: source /opt/ros/jazzy/setup.bash

# Add to ~/.bashrc for permanent (adjust for your ROS version)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

**macOS:**
ROS 2 on macOS requires Docker. See: https://docs.ros.org/en/humble/Installation.html

**Windows:**
Use WSL (Windows Subsystem for Linux) with Ubuntu 22.04, then follow Ubuntu steps above.

---

## Step 2: Find Your Network Addresses

You need 3 addresses to configure the system:

### 2.1 Find Your PC's IP Address

**macOS:**
```bash
ifconfig | grep "inet "
```
**Look for output like:**
```
inet 127.0.0.1 netmask 0xff000000         ← Skip this (localhost)
inet 192.168.1.234 netmask 0xffffff00     ← THIS IS YOUR PC IP!
```

**Linux:**
```bash
ip addr show | grep "inet "
```
**Look for:**
```
inet 192.168.1.234/24 brd 192.168.1.255 scope global dynamic en0
     ^^^^^^^^^^^^^ THIS IS YOUR PC IP!
```

**Windows (for WSL):**
```powershell
# Run in Windows PowerShell (not WSL terminal)
ipconfig
```
**Look for your WiFi adapter:**
```
Wireless LAN adapter Wi-Fi:
   IPv4 Address. . . . . . . . . . . : 192.168.1.234
                                       ^^^^^^^^^^^^^ THIS IS YOUR PC IP!
```

**⚠️ WSL Users:** Use your **Windows WiFi IP**, not WSL's internal IP (172.x.x.x)!

**Common IP formats:**
- `192.168.1.x` (most common)
- `192.168.0.x` (some routers)
- `10.0.0.x` (Apple routers, modern routers, institutional networks)

**Example:** Your PC might be `192.168.1.234`

---

### 2.2 Find Your Quest 3's IP Address

1. Put on your Meta Quest 3
2. Open **Settings** (gear icon)
3. Select **WiFi**
4. Select your connected network name
5. Look for **IP Address**

**Example:** Quest might be `192.168.1.87`

**⚠️ Important:** Quest IP must be on the **same network** as your PC!
- If PC is `192.168.1.x`, Quest should be `192.168.1.y`
- If PC is `10.0.0.x`, Quest should be `10.0.0.y`

---

### 2.3 Find Your Serial Port (ORCA Hand USB)

**Plug in your ORCA Hand via USB first!**

**macOS:**
```bash
ls /dev/tty.usbserial-*
```
**Expected output:**
```
/dev/tty.usbserial-FT9MISJT
```

**Linux:**
```bash
ls /dev/ttyUSB*
```
**Expected output:**
```
/dev/ttyUSB0
```

**Windows:**
1. Open **Device Manager**
2. Expand **Ports (COM & LPT)**
3. Look for **USB Serial Port (COMx)**
4. Note the COM number (e.g., `COM3`)

**⚠️ WSL Users:** You need USB passthrough! See [Section 5: WSL USB Setup](#step-5-wsl-usb-passthrough-windows-only)

---

## Step 3: Configure Your System

Now edit 2 configuration files with YOUR addresses.

### 3.1 Configure Network (Quest ↔ PC Communication)

**File:** `configs/network.yaml`

**Full path examples:**
- Linux/macOS: `<YOUR_REPO_PATH>/OpenTeach_Orca/configs/network.yaml`
- Windows: `<YOUR_REPO_PATH>\OpenTeach_Orca\configs\network.yaml`
- WSL: `/mnt/c/Users/<YourName>/OpenTeach_Orca/configs/network.yaml`

**Edit this file:**
```yaml
# Network configuration for OpenTeach
host_address: "<YOUR_PC_IP>"      # Replace with YOUR PC's IP from Step 2.1
oculus_ip: "<YOUR_QUEST_IP>"      # Replace with YOUR Quest's IP from Step 2.2
host_port: 8087                    # Don't change
zmq_detector_pub_port: 8088        # Don't change
zmq_transform_pub_port: 8089       # Don't change
zmq_vis_pub_port: 15001           # Don't change
```

**Example (with your actual IPs):**
```yaml
host_address: "192.168.1.234"      # YOUR PC's WiFi IP
oculus_ip: "192.168.1.87"          # YOUR Quest's WiFi IP
host_port: 8087
zmq_detector_pub_port: 8088
zmq_transform_pub_port: 8089
zmq_vis_pub_port: 15001
```

**Save the file!**

---

### 3.2 Configure Serial Port (PC → ORCA Hand)

**File:** `orca_core/orca_core/models/orcahand_v1_right/config.yaml`

**Full path examples:**
- Linux/macOS: `<YOUR_REPO_PATH>/OpenTeach_Orca/orca_core/orca_core/models/orcahand_v1_right/config.yaml`
- Windows: `<YOUR_REPO_PATH>\OpenTeach_Orca\orca_core\orca_core\models\orcahand_v1_right\config.yaml`
- WSL: `/mnt/c/Users/<YourName>/OpenTeach_Orca/orca_core/orca_core/models/orcahand_v1_right/config.yaml`

**Find this section:**
```yaml
# Hardware connection settings
version: "1.0"
baudrate: 3000000
port: "<YOUR_SERIAL_PORT>"       # ← Edit this line only!
max_current: 400
type: "right"
control_mode: "current_based_position"
```

**Replace with YOUR port from Step 2.3:**

**macOS example:**
```yaml
port: "/dev/tty.usbserial-FT9MISJT"
```

**Linux example:**
```yaml
port: "/dev/ttyUSB0"
```

**Windows example:**
```yaml
port: "COM3"
```

**Save the file!**

---

### 3.3 Verify Configuration

```bash
# Check network config
cat configs/network.yaml | grep -E "host_address|oculus_ip"
# Should show YOUR IPs, not placeholders!

# Check serial port config
cat orca_core/orca_core/models/orcahand_v1_right/config.yaml | grep "port:"
# Should show YOUR port, not placeholder!
```

---

## Step 4: Build ROS 2 Workspace

The ROS 2 workspace provides the hardware interface node.

### 4.1 Navigate to Workspace

```bash
# Linux/macOS:
cd <YOUR_REPO_PATH>/OpenTeach_Orca/ros2_ws

# Windows (WSL):
cd /mnt/c/Users/<YourName>/OpenTeach_Orca/ros2_ws
```

### 4.2 Source ROS 2

```bash
# Ubuntu/WSL (use your installed ROS version - check with: which ros2):
source /opt/ros/humble/setup.bash
# OR if you have Jazzy:
# source /opt/ros/jazzy/setup.bash

# macOS (if installed locally):
source /path/to/ros2_humble/setup.bash
```

### 4.3 Build with Colcon

```bash
colcon build

# If successful, you'll see:
# Summary: X packages finished [time]
```

### 4.4 Source the Workspace

```bash
source install/setup.bash
```

### 4.5 Verify Build

```bash
ros2 pkg list | grep orca

# Should show:
# orca_hardware_interface
```

**✅ ROS 2 workspace is ready!**

---

## Step 5: WSL USB Passthrough (Windows Only)

**Skip this if you're on Linux/macOS!**

WSL2 doesn't have direct USB access. You need `usbipd-win`.

### 5.1 Install usbipd-win (on Windows)

```powershell
# Run in Windows PowerShell as Administrator
winget install --interactive --exact dorssel.usbipd-win
```

### 5.2 Install USB Tools (in WSL/Ubuntu)

```bash
sudo apt install linux-tools-generic hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20
```

### 5.3 Connect USB Device

**Every time you plug in the ORCA hand:**

```powershell
# In Windows PowerShell as Administrator:

# 1. List USB devices
usbipd list

# 2. Find your ORCA hand (look for "FTDI USB Serial")
# Note the BUSID (e.g., 3-2)

# 3. Bind it (one-time setup per device)
usbipd bind --busid 3-2

# 4. Attach to WSL (do this every time you plug in)
usbipd attach --wsl --busid 3-2
```

**Verify in WSL:**
```bash
ls /dev/ttyUSB*
# Should show: /dev/ttyUSB0
```

### 5.4 Windows Firewall (Allow Quest Communication)

```powershell
# In Windows PowerShell as Administrator:
New-NetFirewallRule -DisplayName "OpenTeach Quest Input" -Direction Inbound -LocalPort 8087 -Protocol UDP -Action Allow
```

**✅ WSL is ready for USB and networking!**

---

## Step 6: Run the System!

Now everything is configured. Time to run!

### 6.1 Start ROS 2 Hardware Node (Terminal 1)

```bash
# Navigate to ROS workspace
cd <YOUR_REPO_PATH>/OpenTeach_Orca/ros2_ws

# Source ROS 2 (use your installed version: humble or jazzy)
source /opt/ros/humble/setup.bash
# OR: source /opt/ros/jazzy/setup.bash

# Source workspace
source install/setup.bash

# Launch hardware node
ros2 launch orca_hardware_interface orca_hardware.launch.py
```

**Expected output:**
```
[INFO] Initializing ORCA Hardware Node...
[INFO] ORCA Hand object created successfully
[INFO] Connecting to ORCA hand...
[INFO] ✓ Connected: Connection successful
[INFO] ✓ Torque enabled on all motors
[INFO] ORCA Hardware Node initialized successfully
```

**⚠️ If connection fails:**
- Check serial port is correct in `orca_core/.../config.yaml`
- Check USB cable is connected
- On Linux: Check permissions (`sudo usermod -a -G dialout $USER`, then log out/in)
- On WSL: Check USB passthrough (see Step 5)

**Leave this terminal running!**

---

### 6.2 Run OpenTeach Teleoperation (Terminal 2)

**Open a NEW terminal:**

```bash
# Navigate to repo root
cd <YOUR_REPO_PATH>/OpenTeach_Orca

# Activate Python environment (if using venv)
source venv/bin/activate

# Run teleoperation
python teleop.py robot=orca
```

**Expected output:**
```
[INFO] Loading config from configs/teleop.yaml
[INFO] Starting OculusVRHandDetector on port 8087
[INFO] Starting TransformHandPositionCoords
[INFO] Starting OrcaOperator at 90 Hz
[INFO] Robot: OrcaHand initialized
[INFO] Waiting for Quest hand data...
```

---

### 6.3 Put on Quest 3 and Move Your Hand!

1. **Put on your Meta Quest 3**
2. **Launch the OpenTeach Unity app** (Quest hand tracking)
3. **Hold your hand in front of you**
4. **Move your fingers** → ORCA hand should follow!

**You should see:**
- Terminal 2: `[INFO] Received hand data, sending to ORCA`
- ORCA hand moving in sync with your hand!

**🎉 Congratulations! You're teloperating the ORCA hand!**

---

## 🔧 Troubleshooting

### Quest Not Sending Data

**Symptoms:** Terminal says "Waiting for Quest hand data..."

**Solutions:**
1. Check Quest is on same WiFi as PC
2. Verify IP addresses in `configs/network.yaml`
3. Test Quest connection:
   ```bash
   python -c "
   from openteach.utils.network import ZMQKeypointSubscriber
   s = ZMQKeypointSubscriber('<YOUR_PC_IP>', 8087, 'hand_coords')
   print('Received:', s.recv_keypoints().shape)
   "
   # Should show: Received: (21, 3)
   ```
4. Check Quest Unity app is running and publishing

---

### ROS Hardware Node Not Connecting

**Symptoms:** `[ERROR] Failed to connect to ORCA hand`

**Solutions:**
1. **Check serial port:**
   ```bash
   ls /dev/tty.usbserial-*  # macOS
   ls /dev/ttyUSB*          # Linux
   ```
2. **Check permissions (Linux):**
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log back in
   groups | grep dialout  # Verify
   ```
3. **WSL: Check USB passthrough:**
   ```powershell
   usbipd list  # In Windows PowerShell
   usbipd attach --wsl --busid <BUSID>
   ```
4. **Test orca_core directly:**
   ```python
   from orca_core import OrcaHand
   hand = OrcaHand()
   success, msg = hand.connect()
   print(f"Connect: {success}, {msg}")
   ```

---

### Robot Moves Erratically

**Symptoms:** ORCA hand jerks or moves incorrectly

**Solutions:**
1. **Calibrate joint gains** in `configs/robot/orca.yaml`:
   ```yaml
   config:
     joint_gain:
       wrist: 0.8          # Reduce wrist sensitivity
       thumb_mcp: 1.2      # Increase thumb curl
   ```
2. **Check handedness** matches your setup:
   ```yaml
   handedness: right  # or "left"
   ```
3. **Verify joint limits** in ORCA config:
   ```bash
   cat orca_core/orca_core/models/orcahand_v1_right/config.yaml
   ```

---

### ROS Topics Not Visible

**Symptoms:** `ros2 topic list | grep orca` shows nothing

**Solutions:**
1. **Source ROS 2:**
   ```bash
   source /opt/ros/humble/setup.bash  # or /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```
2. **Check hardware node is running** (Terminal 1 should show `[INFO] ...`)
3. **Verify package built:**
   ```bash
   ros2 pkg list | grep orca
   # Should show: orca_hardware_interface
   ```

---

### Port Permission Denied (Linux)

**Symptoms:** `Permission denied: '/dev/ttyUSB0'`

**Solution:**
```bash
# Add your user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in (or reboot)

# Verify
groups | grep dialout
```

---

### Quest and PC on Different Networks

**Symptoms:** Quest IP is `10.0.0.x` but PC is `192.168.1.x`

**Solution:**
- **Both devices must be on same WiFi network**
- Reconnect Quest to the same WiFi as your PC
- Check both IPs again (Step 2)

---

## 🎯 Next Steps

### Collect Demonstration Data
```bash
python data_collect.py robot=orca
# Perform task with Quest tracking
# Data saved to demonstration_N/ folder
```

### Enable Visualization
```yaml
# Edit configs/teleop.yaml
visualize_right_2d: true
```
```bash
python teleop.py robot=orca
# Opens matplotlib window with hand skeleton
```

### Tune Sensitivity
```yaml
# Edit configs/robot/orca.yaml
config:
  joint_gain:
    wrist: 0.8              # Reduce wrist movement
    index_mcp: 1.2          # Increase index curl
  joint_bias:
    thumb_abd: 0.1          # Add offset to thumb abduction
```

### Switch to Left Hand
```yaml
# Edit configs/robot/orca.yaml
config:
  handedness: left
```

### Train Imitation Learning Policies
- Use recorded `joint_states.txt` and `commanded_joint_states.txt`
- Train policy with your favorite IL method
- Deploy with `deploy_server.py`

---

## 📊 System Summary

**What you've built:**
```
👋 Your Hand → Quest 3 (MediaPipe 90Hz)
    ↓ WiFi (port 8087)
OpenTeach Detector
    ↓ ZMQ (localhost:8088)
OpenTeach Transform
    ↓ ZMQ (localhost:8089)
OpenTeach Operator (Geometric Retargeting)
    ↓ ROS 2 (/orca_hand/command)
ROS Hardware Node (orca_core)
    ↓ Serial USB (3 Mbaud)
🤖 ORCA Hand (17 motors)
```

**Network Requirements:**
- Quest and PC on **same WiFi network**
- UDP port **8087** open for Quest → PC
- ZMQ ports **8088, 8089** on localhost (automatically work)
- (Optional) Port **15001** for visualizer → Quest

**Serial Requirements:**
- USB connection: PC ↔ ORCA hand
- Baudrate: **3000000** (3 Mbaud)
- Protocol: Dynamixel Protocol 2.0
- On Linux: User in `dialout` group
- On WSL: USB passthrough with usbipd-win

---

## 📚 Additional Resources

- **SYSTEM_ARCHITECTURE.md** - Complete technical reference
- **ros2_ws/README.md** - ROS 2 workspace details
- **add_your_own_robot.md** - Add new robots to OpenTeach
- **teleop_data_collect.md** - Data collection guide
- **vr.md** - Quest VR setup details

---

## ✅ Configuration Checklist

Before running, verify:

- [ ] Repository cloned with `--recursive`
- [ ] OpenTeach installed: `pip install -e .`
- [ ] orca_core installed: `cd orca_core && pip install -e .`
- [ ] ROS 2 (Humble or Jazzy) installed
- [ ] ROS 2 workspace built: `colcon build`
- [ ] PC IP address found and configured in `configs/network.yaml`
- [ ] Quest IP address found and configured in `configs/network.yaml`
- [ ] Serial port found and configured in `orca_core/.../config.yaml`
- [ ] (Linux) User added to `dialout` group
- [ ] (WSL) usbipd-win installed and USB attached
- [ ] (WSL) Windows Firewall allows port 8087
- [ ] ORCA hand connected via USB
- [ ] Quest and PC on same WiFi network
- [ ] Quest Unity app installed and ready
