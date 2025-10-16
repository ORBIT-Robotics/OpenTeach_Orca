# OpenTeach ORCA Networking - Complete Explanation

## **Overview: Same PC Setup (Your Configuration)**

When **both Meta Quest 3 AND ORCA Hand are connected to the SAME PC**, here's the complete picture:

```
┌────────────────────────────────────────────────────────────────┐
│                    YOUR PC (192.168.1.100)                     │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  ┌──────────────────────────────────────────────────────┐     │
│  │         Meta Quest 3 (WiFi Connected)                │     │
│  │         IP: 192.168.1.50                             │     │
│  └──────────────────────────────────────────────────────┘     │
│                          ↓                                     │
│              WiFi Network (192.168.1.x)                        │
│                          ↓                                     │
│  ┌──────────────────────────────────────────────────────┐     │
│  │  Process 1: OculusVRHandDetector                     │     │
│  │  - Listens on: 0.0.0.0:8087 (ALL interfaces)         │     │
│  │  - Receives: MediaPipe keypoints from Quest          │     │
│  │  - Publishes to: ZMQ port 8088                       │     │
│  └──────────────────────────────────────────────────────┘     │
│                          ↓                                     │
│  ┌──────────────────────────────────────────────────────┐     │
│  │  Process 2: TransformHandPositionCoords              │     │
│  │  - Subscribes from: ZMQ port 8088                    │     │
│  │  - Transforms coordinates                            │     │
│  │  - Publishes to: ZMQ port 8089                       │     │
│  └──────────────────────────────────────────────────────┘     │
│                          ↓                                     │
│  ┌──────────────────────────────────────────────────────┐     │
│  │  Process 3: OrcaOperator                             │     │
│  │  - Subscribes from: ZMQ port 8089                    │     │
│  │  - Retargets: 21 keypoints → 17 joint angles         │     │
│  │  - Publishes to: localhost ROS topic                 │     │
│  │    /orca_hand/command                                │     │
│  └──────────────────────────────────────────────────────┘     │
│                          ↓                                     │
│              ROS 2 (localhost communication)                   │
│                          ↓                                     │
│  ┌──────────────────────────────────────────────────────┐     │
│  │  Process 4: orca_hardware_node (ROS 2)               │     │
│  │  - Subscribes from: /orca_hand/command               │     │
│  │  - Uses: orca_core.OrcaHand                          │     │
│  │  - Sends to: Serial port (USB)                       │     │
│  │  - Publishes: /orca_hand/joint_states                │     │
│  └──────────────────────────────────────────────────────┘     │
│                          ↓                                     │
│              Serial/USB Connection                             │
│                          ↓                                     │
│  ┌──────────────────────────────────────────────────────┐     │
│  │         ORCA Hand (USB Connected)                    │     │
│  │         Port: /dev/tty.usbserial-XXXXXXX             │     │
│  │         17 Dynamixel Motors                          │     │
│  └──────────────────────────────────────────────────────┘     │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

---

## **🔌 Connection Types Explained**

### **1. WiFi Connection (Quest → PC)**

**What it is:**
- Meta Quest 3 connects to your **local WiFi network**
- Both Quest and PC get IP addresses from your router

**IP Addresses:**
- **Your PC:** Let's say `192.168.1.100` (assigned by router)
- **Quest 3:** Let's say `192.168.1.50` (assigned by router)
- **Router:** Usually `192.168.1.1`

**Configuration in OpenTeach:**
```yaml
# configs/network.yaml
host_address: "192.168.1.100"  # Your PC's IP
oculus_ip: "192.168.1.50"      # Quest's IP
```

**How it works:**
1. Quest 3 runs an app that streams hand tracking data
2. App sends UDP packets to `192.168.1.100:8087`
3. OculusVRHandDetector on your PC listens on port `8087`
4. Data flows over your WiFi network

---

### **2. ZMQ Ports (Localhost - Same PC)**

**What is ZMQ?**
- **Z**ero**MQ** = High-performance messaging library
- Used for **inter-process communication** on the same machine
- Think of it as "pipes between programs"

**Port Numbers:**
- `8087` - OculusVRHandDetector listens (from Quest)
- `8088` - Detector publishes, Transform subscribes
- `8089` - Transform publishes, Operator subscribes
- `15001` - Visualizer feedback (optional)

**Why different ports?**
Each process needs its own "door number" to avoid conflicts.

**Localhost vs 0.0.0.0:**
- `localhost` or `127.0.0.1` = "this computer only"
- `0.0.0.0` = "listen on ALL network interfaces" (WiFi + localhost)

**Configuration:**
```yaml
# configs/network.yaml
zmq_detector_pub_port: 8088      # Detector → Transform
zmq_transform_pub_port: 8089     # Transform → Operator
```

**How data flows:**
```python
# Detector publishes:
socket.bind("tcp://0.0.0.0:8088")
socket.send(keypoint_data)

# Transform subscribes:
socket.connect("tcp://localhost:8088")
data = socket.recv()
```

---

### **3. ROS 2 Topics (Localhost - Same PC)**

**What is ROS 2?**
- **R**obot **O**perating **S**ystem
- Middleware for robot communication
- Uses **DDS** (Data Distribution Service) under the hood

**Topics:**
- `/orca_hand/command` - OpenTeach → Hardware Node
- `/orca_hand/joint_states` - Hardware Node → OpenTeach

**How it works on same PC:**
```python
# In orca_control.py (OpenTeach):
publisher = node.create_publisher('/orca_hand/command')
publisher.publish(joint_angles)

# In orca_hardware_node.py:
subscriber = node.create_subscription('/orca_hand/command', callback)
# callback receives the data
```

**Behind the scenes:**
- ROS 2 uses **shared memory** when on same PC (super fast!)
- No network configuration needed for localhost
- All processes see each other automatically

---

### **4. Serial/USB Connection (Hardware)**

**What it is:**
- Physical USB cable from PC to ORCA hand
- Creates a "virtual serial port"

**Port Names:**
- **macOS:** `/dev/tty.usbserial-FT9MISJT`
- **Linux:** `/dev/ttyUSB0`
- **Windows:** `COM3`

**Configuration:**
```yaml
# orca_core/models/orcahand_v1_right/config.yaml
port: "/dev/tty.usbserial-FT9MISJT"
baudrate: 3000000  # 3 Mbps
```

**How it works:**
```python
# In orca_core:
serial_connection = DynamixelClient(port="/dev/tty.usbserial-XXX")
serial_connection.send_command(motor_id, position)
```

---

## **📊 Complete Data Flow - Same PC**

### **Timeline of One Control Loop (90 Hz = 11ms per loop)**

```
t = 0ms:   Quest captures hand pose
           └─> Sends UDP packet to 192.168.1.100:8087

t = 1ms:   OculusVRHandDetector receives packet
           ├─> Extracts 21 keypoints (x,y,z each)
           └─> Publishes to ZMQ port 8088

t = 2ms:   TransformHandPositionCoords receives
           ├─> Normalizes coordinates
           └─> Publishes to ZMQ port 8089

t = 3ms:   OrcaOperator receives
           ├─> Runs quest21_to_orca17() retargeting
           ├─> Computes 17 joint angles
           └─> Publishes to ROS /orca_hand/command

t = 4ms:   orca_hardware_node receives
           ├─> Converts radians → degrees
           ├─> Sends serial command to motors
           └─> Reads feedback from motors

t = 5ms:   Motors move to new position
           └─> Send position feedback

t = 6ms:   orca_hardware_node publishes
           └─> ROS /orca_hand/joint_states

t = 7ms:   orca_control.py receives feedback
           └─> Updates internal state

t = 11ms:  Next Quest frame arrives (90 Hz)
```

---

## **🌐 IP Addresses & Ports - Complete Reference**

### **What's an IP Address?**
Think of it as a **street address** for devices on a network.

**Format:** `192.168.1.100`
- `192.168.1.x` = Your local network (like "Main Street")
- Last number (`100`) = Specific device (like "House #100")

### **What's a Port?**
Think of it as an **apartment number** at that address.

**Example:** `192.168.1.100:8087`
- IP = Street address
- Port = Apartment number (which program to talk to)

### **Special IP Addresses:**

| IP Address | Meaning |
|------------|---------|
| `127.0.0.1` | Localhost (this computer only) |
| `0.0.0.0` | All interfaces (WiFi + localhost) |
| `192.168.1.1` | Usually your router |
| `192.168.1.x` | Devices on your local network |

### **Port Numbers in Your System:**

| Port | Protocol | Purpose | Direction |
|------|----------|---------|-----------|
| `8087` | UDP | Quest → Detector | IN (from WiFi) |
| `8088` | ZMQ | Detector → Transform | Internal (localhost) |
| `8089` | ZMQ | Transform → Operator | Internal (localhost) |
| `15001` | ZMQ | Visualizer feedback | Internal (localhost) |
| ROS topics | DDS | Operator ↔ Hardware | Internal (shared memory) |

---

## **🔧 Configuration Files - What to Change**

### **When Everything is on Same PC:**

**1. Find Your PC's IP Address:**
```bash
# macOS
ifconfig | grep "inet "

# Linux
ip addr show

# Look for something like: 192.168.1.100
```

**2. Find Your Quest's IP Address:**
- Go to Quest Settings → WiFi → Your Network
- Look for IP address (e.g., `192.168.1.50`)

**3. Edit OpenTeach Config:**
```yaml
# configs/network.yaml
host_address: "192.168.1.100"      # Your PC's IP (what Quest connects to)
oculus_ip: "192.168.1.50"          # Quest's IP (not really used if Quest sends to PC)
host_port: 8087                    # Where Detector listens

zmq_detector_pub_port: 8088        # No change needed (localhost)
zmq_transform_pub_port: 8089       # No change needed (localhost)
```

**4. ROS 2 - No Configuration Needed!**
- When on same PC, ROS 2 auto-discovers nodes
- Uses shared memory (faster than network)

---

## **❓ Common Questions**

### **Q: What if I change WiFi networks?**
**A:** You'll need to:
1. Find new PC IP: `ifconfig`
2. Find new Quest IP: Quest Settings
3. Update `configs/network.yaml`

### **Q: Can I use localhost (127.0.0.1) for Quest?**
**A:** NO! Quest is a separate device.
- Use `localhost` only for same-PC communication (ZMQ, ROS)
- Use `192.168.1.x` for Quest → PC communication

### **Q: Why does Detector listen on 0.0.0.0 and not localhost?**
**A:** Because Quest sends data **over WiFi**:
- `0.0.0.0:8087` = Accept from WiFi or localhost
- `localhost:8087` = Only accept from same PC (Quest can't connect!)

### **Q: What if ports are already in use?**
**A:** Change port numbers in configs:
```yaml
host_port: 8187  # Changed from 8087
zmq_detector_pub_port: 8188  # Changed from 8088
# etc.
```

### **Q: Do I need to configure anything for serial connection?**
**A:** Yes, in orca_core config:
```yaml
# orca_core/models/orcahand_v1_right/config.yaml
port: "/dev/tty.usbserial-YOUR_SERIAL"
```

Find your port:
```bash
# macOS
ls /dev/tty.usbserial-*

# Linux
ls /dev/ttyUSB*
```

---

## **🎯 Summary: Same PC Setup**

**Physical Connections:**
1. ✅ Quest 3 → WiFi → Router → PC WiFi (wireless)
2. ✅ ORCA Hand → USB cable → PC USB port (wired)

**Software Communication:**
1. ✅ Quest → PC WiFi: Uses IP `192.168.1.x` and port `8087`
2. ✅ Between OpenTeach processes: ZMQ on localhost ports `8088`, `8089`
3. ✅ OpenTeach ↔ ROS node: ROS 2 topics on localhost (shared memory)
4. ✅ ROS node → Motors: Serial port `/dev/tty.usbserial-XXX`

**What YOU need to configure:**
- ✅ `configs/network.yaml` - Set your PC and Quest IPs
- ✅ `orca_core/.../config.yaml` - Set correct serial port
- ❌ ROS 2 topics - Auto-configured for localhost!
- ❌ ZMQ ports - Already set in configs!

---

## **🚀 Quick Setup Checklist**

- [ ] 1. Connect Quest 3 to WiFi
- [ ] 2. Connect ORCA hand via USB
- [ ] 3. Find PC IP: `ifconfig`
- [ ] 4. Find Quest IP: Quest Settings
- [ ] 5. Find serial port: `ls /dev/tty.usbserial-*`
- [ ] 6. Edit `configs/network.yaml` with IPs
- [ ] 7. Edit `orca_core/.../config.yaml` with serial port
- [ ] 8. Test: `ros2 launch orca_hardware_interface orca_hardware.launch.py`
- [ ] 9. Run: `python teleop.py robot=orca`

---

**Still confused?** Think of it like a **relay race**:
1. Quest → throws data over WiFi to PC
2. Detector → catches it, passes to Transform via ZMQ
3. Transform → processes, passes to Operator via ZMQ
4. Operator → retargets, passes to ROS node via ROS topic
5. ROS node → sends to motors via USB serial

Each handoff uses a different "communication method" but they all happen on your PC! 🏃‍♂️→🏃‍♀️→🏃→🏃‍♂️→🤖
