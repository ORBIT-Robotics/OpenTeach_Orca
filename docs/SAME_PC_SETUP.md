# Same PC Setup - Visual Diagram

## Your Complete System on One Computer

```
                    SAME PC (192.168.1.100)
┌──────────────────────────────────────────────────────────────────┐
│                                                                  │
│  ╔════════════════════════════════════════════════════════╗     │
│  ║  Meta Quest 3 (Connected via WiFi)                    ║     │
│  ║  IP: 192.168.1.50                                     ║     │
│  ║  - Hand tracking at 90 Hz                             ║     │
│  ║  - MediaPipe generates 21 keypoints                   ║     │
│  ╚════════════════════════════════════════════════════════╝     │
│                            │                                     │
│                            │ WiFi (UDP packets)                  │
│                            │ Destination: 192.168.1.100:8087     │
│                            ↓                                     │
│  ┌────────────────────────────────────────────────────────┐     │
│  │ LAYER 1: OculusVRHandDetector (Python Process)        │     │
│  │ • Listens: 0.0.0.0:8087                               │     │
│  │ • Receives: 21 keypoints (x,y,z) from Quest           │     │
│  │ • Frequency: 90 Hz                                    │     │
│  └────────────────────────────────────────────────────────┘     │
│                            │                                     │
│                            │ ZMQ (localhost)                     │
│                            │ tcp://localhost:8088                │
│                            ↓                                     │
│  ┌────────────────────────────────────────────────────────┐     │
│  │ LAYER 2: TransformHandPositionCoords (Python Process) │     │
│  │ • Subscribes: ZMQ port 8088                           │     │
│  │ • Normalizes coordinates                              │     │
│  │ • Publishes: ZMQ port 8089                            │     │
│  └────────────────────────────────────────────────────────┘     │
│                            │                                     │
│                            │ ZMQ (localhost)                     │
│                            │ tcp://localhost:8089                │
│                            ↓                                     │
│  ┌────────────────────────────────────────────────────────┐     │
│  │ LAYER 3: OrcaOperator (Python Process)                │     │
│  │ • Subscribes: ZMQ port 8089                           │     │
│  │ • Retargeting: quest21_to_orca17()                    │     │
│  │   - 21 keypoints → 17 joint angles                    │     │
│  │   - Palm-frame geometric computation                  │     │
│  │   - Wrist angle from gravity vector                   │     │
│  │ • Output: 17 joint angles (radians)                   │     │
│  └────────────────────────────────────────────────────────┘     │
│                            │                                     │
│            ╔═══════════════╩═══════════════╗                    │
│            ║      ROS 2 MIDDLEWARE         ║                    │
│            ║   (Shared Memory - Fast!)     ║                    │
│            ╚═══════════════╦═══════════════╝                    │
│                            │                                     │
│          Topic: /orca_hand/command (Float64MultiArray)          │
│                            │                                     │
│                            ↓                                     │
│  ┌────────────────────────────────────────────────────────┐     │
│  │ LAYER 4: orca_hardware_node (ROS 2 Node)              │     │
│  │ • Subscribes: /orca_hand/command                      │     │
│  │ • Uses: orca_core.OrcaHand class                      │     │
│  │ • Converts: radians → degrees                         │     │
│  │ • Sends: Serial commands to motors                    │     │
│  │ • Publishes: /orca_hand/joint_states (feedback)       │     │
│  └────────────────────────────────────────────────────────┘     │
│                            │                                     │
│                            │ Serial/USB                          │
│                            │ /dev/tty.usbserial-XXX              │
│                            │ Baudrate: 3000000                   │
│                            ↓                                     │
│  ┌────────────────────────────────────────────────────────┐     │
│  │ ORCA Hand Hardware (USB Connected)                    │     │
│  │ • 17 Dynamixel Servos                                 │     │
│  │   - Thumb: 4 joints (MCP, ABD, PIP, DIP)              │     │
│  │   - Index: 3 joints (MCP, PIP, DIP)                   │     │
│  │   - Middle: 3 joints (MCP, PIP, DIP)                  │     │
│  │   - Ring: 3 joints (MCP, PIP, DIP)                    │     │
│  │   - Pinky: 3 joints (MCP, PIP, DIP)                   │     │
│  │   - Wrist: 1 joint (flexion)                          │     │
│  └────────────────────────────────────────────────────────┘     │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

## Protocol Summary

| Connection | Type | Address | Speed | Purpose |
|------------|------|---------|-------|---------|
| Quest → PC | WiFi (UDP) | 192.168.1.100:8087 | 90 Hz | Hand tracking data |
| Detector → Transform | ZMQ | localhost:8088 | 90 Hz | Raw keypoints |
| Transform → Operator | ZMQ | localhost:8089 | 90 Hz | Normalized keypoints |
| Operator → ROS Node | ROS Topic | /orca_hand/command | 90 Hz | Joint angles |
| ROS Node → Hardware | Serial | /dev/ttyUSB0 | 3 Mbps | Motor commands |

## Port Configuration

**Edit: `configs/network.yaml`**
```yaml
host_address: "192.168.1.100"      # YOUR PC's WiFi IP
oculus_ip: "192.168.1.50"          # Quest's WiFi IP
host_port: 8087                    # Where to listen for Quest
zmq_detector_pub_port: 8088        # Detector → Transform
zmq_transform_pub_port: 8089       # Transform → Operator
```

**Edit: `orca_core/models/orcahand_v1_right/config.yaml`**
```yaml
port: "/dev/tty.usbserial-FT9MISJT"  # YOUR serial port
baudrate: 3000000
```

## What Runs Where

**Terminal 1:** ROS 2 Hardware Node
```bash
ros2 launch orca_hardware_interface orca_hardware.launch.py
```

**Terminal 2:** OpenTeach (includes all 4 Python processes)
```bash
python teleop.py robot=orca
```

**Behind the scenes:** OpenTeach uses `multiprocessing` to spawn:
- Process 1: OculusVRHandDetector
- Process 2: TransformHandPositionCoords
- Process 3: OrcaOperator (with orca_control.py ROS client)
- Process 4: Hand2DVisualizer (optional, port 15001)

All communicate via ZMQ and ROS 2 on the **same PC**!
