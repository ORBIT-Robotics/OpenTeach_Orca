# Configuration Guide - Finding YOUR Addresses

This guide helps you find and configure the correct IP addresses and ports for YOUR specific setup.

## 🔍 **Step 1: Find Your PC's IP Address**

### **macOS:**
```bash
ifconfig | grep "inet "
```

**Look for output like:**
```
inet 127.0.0.1 netmask 0xff000000         ← Skip this (localhost)
inet 192.168.1.234 netmask 0xffffff00     ← THIS IS YOUR IP!
```

**Or check specific WiFi interface:**
```bash
ifconfig en0 | grep "inet "
```

### **Linux:**
```bash
ip addr show | grep "inet "
```

**Look for output like:**
```
inet 192.168.1.234/24 brd 192.168.1.255 scope global dynamic en0
     ^^^^^^^^^^^^^ THIS IS YOUR IP!
```

### **Windows (for WSL/Ubuntu):**

**Option 1 - From Windows PowerShell/CMD:**
```powershell
ipconfig
```

**Look for your WiFi adapter output:**
```
Wireless LAN adapter Wi-Fi:
   IPv4 Address. . . . . . . . . . . : 192.168.1.234
                                       ^^^^^^^^^^^^^ THIS IS YOUR IP!
```

**Option 2 - From WSL/Ubuntu terminal:**
```bash
# Get Windows host IP from WSL
ip route show | grep -i default | awk '{ print $3}'
```

**Or check all network interfaces:**
```bash
# This shows WSL's view of Windows network
cat /etc/resolv.conf | grep nameserver
```

**Important for WSL:**
- Use your **Windows machine's WiFi IP** (from `ipconfig`), not WSL's internal IP
- WSL uses a virtual network adapter with IPs like `172.x.x.x` - don't use these!
- Your Quest needs to connect to the **Windows WiFi IP** on your physical network

### **Common IP Formats:**
- `192.168.1.x` (most common)
- `192.168.0.x` (some routers)
- `10.0.0.x` (Apple routers, some modern routers)
- `172.16.x.x` (less common for WiFi)

**Example:** Your Windows PC might be `192.168.1.234` or `10.0.0.45`

**⚠️ WSL Users:** Don't confuse WSL's internal IPs (172.x.x.x) with your actual WiFi IP!

---

## 📱 **Step 2: Find Your Quest 3's IP Address**

1. Put on your Meta Quest 3
2. Open **Settings** (gear icon)
3. Select **WiFi**
4. Select your connected network name
5. Look for **IP Address**

**Example:** Quest might be `192.168.1.87` or `10.0.0.52`

**Important:** Quest IP must be on the **same network** as your PC!
- If PC is `192.168.1.x`, Quest should be `192.168.1.y`
- If PC is `10.0.0.x`, Quest should be `10.0.0.y`

---

## 🔌 **Step 3: Find Your Serial Port**

### **macOS:**
```bash
ls /dev/tty.usbserial-*
```

**Expected output:**
```
/dev/tty.usbserial-FT9MISJT
```

### **Linux:**
```bash
ls /dev/ttyUSB*
```

**Expected output:**
```
/dev/ttyUSB0
```

### **Windows:**
1. Open **Device Manager**
2. Expand **Ports (COM & LPT)**
3. Look for **USB Serial Port (COMx)**

**Example:** Might be `COM3` or `COM4`

---

## ✏️ **Step 4: Edit Configuration Files**

### **File 1: `configs/network.yaml`**

**Location:** `OpenTeach_Orca/configs/network.yaml`

**Before (placeholders):**
```yaml
host_address: "<YOUR_PC_IP>"
oculus_ip: "<YOUR_QUEST_IP>"
```

**After (with YOUR actual IPs):**
```yaml
host_address: "192.168.1.234"  # Replace with YOUR PC's IP from Step 1
oculus_ip: "192.168.1.87"      # Replace with YOUR Quest's IP from Step 2
```

**Full file should look like:**
```yaml
# Network configuration for OpenTeach
host_address: "192.168.1.234"      # YOUR PC's WiFi IP
oculus_ip: "192.168.1.87"          # YOUR Quest's WiFi IP
host_port: 8087                    # Don't change
zmq_detector_pub_port: 8088        # Don't change
zmq_transform_pub_port: 8089       # Don't change
zmq_vis_pub_port: 15001           # Don't change
```

---

### **File 2: `orca_core/models/orcahand_v1_right/config.yaml`**

**Location:** `OpenTeach_Orca/orca_core/orca_core/models/orcahand_v1_right/config.yaml`

**Before (placeholder):**
```yaml
port: "<YOUR_SERIAL_PORT>"
```

**After (with YOUR actual port):**

**macOS example:**
```yaml
port: "/dev/tty.usbserial-FT9MISJT"  # Replace with YOUR port from Step 3
```

**Linux example:**
```yaml
port: "/dev/ttyUSB0"  # Replace with YOUR port from Step 3
```

**Full section should look like:**
```yaml
# Hardware connection settings
version: "1.0"
baudrate: 3000000
port: "/dev/tty.usbserial-FT9MISJT"  # YOUR serial port
max_current: 400
type: "right"
control_mode: "current_based_position"
```

---

## ✅ **Step 5: Verify Configuration**

### **Check Network Config:**
```bash
cat configs/network.yaml | grep -E "host_address|oculus_ip"
```

**Should show YOUR IPs, not placeholders:**
```yaml
host_address: "192.168.1.234"
oculus_ip: "192.168.1.87"
```

### **Check Serial Port Config:**
```bash
cat orca_core/orca_core/models/orcahand_v1_right/config.yaml | grep "port:"
```

**Should show YOUR port, not placeholder:**
```yaml
port: "/dev/tty.usbserial-FT9MISJT"
```

---

## 🔄 **When Do I Need to Update These?**

### **Update Network IPs if:**
- ❌ You connect to a different WiFi network
- ❌ Your router assigns new IPs (after reboot)
- ❌ You move to a different location

**Quick check:**
```bash
# Run ifconfig again to see if IP changed
ifconfig | grep "inet " | grep -v "127.0.0.1"
```

### **Update Serial Port if:**
- ❌ You plug ORCA hand into different USB port
- ❌ You use different computer
- ❌ On Linux, port name changes after reboot

**Quick check:**
```bash
# See if your port still exists
ls /dev/tty.usbserial-* || ls /dev/ttyUSB*
```

---

## 🆘 **Troubleshooting**

### **"Can't find my PC's IP"**
**Solution:**
```bash
# macOS - show all network interfaces
ifconfig

# Linux
ip addr show

# Look for interface that's connected (usually en0, en1, wlan0, or eth0)
# Find the line with "inet" that's NOT 127.0.0.1
```

### **"Quest and PC on different networks"**
**Problem:** PC shows `192.168.1.x` but Quest shows `10.0.0.x`

**Solution:**
- Both devices must be on same WiFi network
- Reconnect Quest to the same WiFi as PC
- Check both IPs again

### **"Serial port not found"**
**macOS:**
```bash
# List ALL USB devices
ls /dev/tty.*

# If nothing with "usbserial", check if device is connected:
system_profiler SPUSBDataType | grep -A 10 "FTDI"
```

**Linux:**
```bash
# List all serial devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# Check dmesg for USB events
dmesg | grep tty
```

**Windows:**
- Device Manager → Ports (COM & LPT)
- If not listed, install FTDI drivers

### **"Port permission denied" (Linux/WSL)**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in, then verify
groups | grep dialout
```

### **"WSL can't access USB port"**
**Problem:** WSL2 doesn't have direct USB access by default

**Solution - Install usbipd-win:**

**Step 1: On Windows (PowerShell as Administrator):**
```powershell
# Install usbipd-win
winget install --interactive --exact dorssel.usbipd-win
```

**Step 2: In WSL/Ubuntu:**
```bash
# Install usbip tools
sudo apt install linux-tools-generic hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20
```

**Step 3: Connect USB device (on Windows PowerShell as Administrator):**
```powershell
# List USB devices
usbipd list

# Find your ORCA hand (look for FTDI USB Serial)
# Bind it (one-time setup, replace <BUSID> with actual bus ID like 3-2)
usbipd bind --busid <BUSID>

# Attach to WSL (do this every time you connect the device)
usbipd attach --wsl --busid <BUSID>
```

**Step 4: Verify in WSL:**
```bash
# Should now see the device
ls /dev/ttyUSB*
```

**Reference:** https://learn.microsoft.com/en-us/windows/wsl/connect-usb

### **"Quest can't reach PC on WSL"**
**Problem:** Windows Firewall blocking port 8087

**Solution:**
```powershell
# On Windows PowerShell as Administrator
# Allow port 8087 for incoming connections
New-NetFirewallRule -DisplayName "OpenTeach Quest Input" -Direction Inbound -LocalPort 8087 -Protocol UDP -Action Allow
```

---

## 📋 **Quick Reference**

| What | Command | Example Output |
|------|---------|----------------|
| PC IP (macOS) | `ifconfig \| grep "inet "` | `inet 192.168.1.234` |
| PC IP (Linux) | `ip addr show \| grep "inet "` | `inet 192.168.1.234/24` |
| PC IP (Windows) | `ipconfig` | `IPv4 Address: 192.168.1.234` |
| PC IP (WSL) | Get Windows IP with `ipconfig` from PowerShell | Use Windows host IP! |
| Quest IP | Quest Settings → WiFi | `192.168.1.87` |
| Serial Port (macOS) | `ls /dev/tty.usbserial-*` | `/dev/tty.usbserial-FT9MISJT` |
| Serial Port (Linux/WSL) | `ls /dev/ttyUSB*` | `/dev/ttyUSB0` |

---

## 🎯 **Summary**

**Replace these placeholders:**
1. `<YOUR_PC_IP>` → Your actual PC IP (e.g., `192.168.1.234`)
2. `<YOUR_QUEST_IP>` → Your actual Quest IP (e.g., `192.168.1.87`)
3. `<YOUR_SERIAL_PORT>` → Your actual serial port (e.g., `/dev/tty.usbserial-FT9MISJT`)

**In these files:**
1. `configs/network.yaml` (IPs)
2. `orca_core/orca_core/models/orcahand_v1_right/config.yaml` (serial port)

**Don't change:**
- Port numbers: `8087`, `8088`, `8089`, `15001`
- Baudrate: `3000000`
- Any `localhost` or `127.0.0.1` addresses
