#!/usr/bin/env python3
"""
orca_hardware_node.py
=====================
ROS 2 hardware interface node for ORCA Hand.

This node:
1. Subscribes to /orca_hand/command (Float64MultiArray) from OpenTeach
2. Uses orca_core.OrcaHand to send commands to physical hardware
3. Publishes /orca_hand/joint_states (JointState) for feedback to OpenTeach

Architecture:
    OpenTeach → orca_control.py (ROS client) → /orca_hand/command topic
                                                        ↓
                                                  [THIS NODE]
                                                        ↓
                                            orca_core.OrcaHand → Motors
                                                        ↓
                                            /orca_hand/joint_states topic
                                                        ↓
                                                orca_control.py → OpenTeach

Author: OpenTeach ORCA Integration
Date: October 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import threading
import sys
import os

# Import orca_core - must be installed in environment
try:
    from orca_core import OrcaHand
except ImportError as e:
    print(f"ERROR: Could not import orca_core: {e}")
    print("\nTo fix this, install orca_core in your Python environment:")
    print("  cd /path/to/OpenTeach_Orca/orca_core")
    print("  pip install -e .")
    print("\nMake sure you're in the correct Python environment!")
    sys.exit(1)


class OrcaHardwareNode(Node):
    """
    ROS 2 node that interfaces between ROS topics and ORCA hardware.
    
    This node runs the actual hardware control loop using orca_core.OrcaHand
    and provides ROS 2 topic interfaces for command input and state feedback.
    """
    
    def __init__(self):
        super().__init__('orca_hardware_node')
        
        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('control_frequency', 90.0)  # Match OpenTeach rate
        self.declare_parameter('auto_connect', True)
        self.declare_parameter('auto_calibrate', False)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.control_frequency = self.get_parameter('control_frequency').value
        auto_connect = self.get_parameter('auto_connect').value
        auto_calibrate = self.get_parameter('auto_calibrate').value
        
        self.get_logger().info(f'Initializing ORCA Hardware Node...')
        self.get_logger().info(f'Model path: {model_path if model_path else "default"}')
        self.get_logger().info(f'Control frequency: {self.control_frequency} Hz')
        
        # Initialize ORCA hand
        try:
            if model_path:
                self.hand = OrcaHand(model_path)
            else:
                # Use default model path from orca_core
                self.hand = OrcaHand()
            self.get_logger().info('ORCA Hand object created successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to create OrcaHand: {e}')
            raise
        
        # Connection state
        self.connected = False
        self.calibrated = False
        
        # Auto-connect if requested
        if auto_connect:
            self.connect_hardware()
            
            # Auto-calibrate if requested
            if auto_calibrate and self.connected:
                self.calibrate_hardware()
        
        # Thread-safe state storage
        self._lock = threading.Lock()
        self._last_command = None
        self._current_joint_states = None
        
        # ROS 2 Subscriber: Listen to commands from OpenTeach
        self.command_sub = self.create_subscription(
            Float64MultiArray,
            '/orca_hand/command',
            self.command_callback,
            10
        )
        
        # ROS 2 Publisher: Publish joint states for feedback
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/orca_hand/joint_states',
            10
        )
        
        # Create timer for publishing joint states at control frequency
        timer_period = 1.0 / self.control_frequency  # seconds
        self.state_timer = self.create_timer(timer_period, self.publish_joint_states)
        
        self.get_logger().info('ORCA Hardware Node initialized successfully')
        self.get_logger().info(f'Subscribed to: /orca_hand/command')
        self.get_logger().info(f'Publishing to: /orca_hand/joint_states')
    
    def connect_hardware(self):
        """Connect to ORCA hand hardware via orca_core."""
        try:
            self.get_logger().info('Connecting to ORCA hand...')
            success, message = self.hand.connect()
            
            if success:
                self.connected = True
                self.get_logger().info(f'✓ Connected: {message}')
                
                # Enable torque on all motors
                self.hand.enable_torque()
                self.get_logger().info('✓ Torque enabled on all motors')
                
            else:
                self.get_logger().error(f'✗ Connection failed: {message}')
                
        except Exception as e:
            self.get_logger().error(f'Exception during connection: {e}')
    
    def calibrate_hardware(self):
        """Run calibration routine on ORCA hand."""
        if not self.connected:
            self.get_logger().warn('Cannot calibrate: not connected to hardware')
            return
        
        try:
            self.get_logger().info('Starting calibration (this may take a while)...')
            self.hand.calibrate()
            self.calibrated = True
            self.get_logger().info('✓ Calibration complete')
            
        except Exception as e:
            self.get_logger().error(f'Calibration failed: {e}')
    
    def command_callback(self, msg: Float64MultiArray):
        """
        Callback for receiving joint position commands from OpenTeach.
        
        Args:
            msg: Float64MultiArray with 17 joint angles in radians
        """
        if not self.connected:
            self.get_logger().warn('Received command but not connected to hardware', throttle_duration_sec=5.0)
            return
        
        # Store command
        with self._lock:
            self._last_command = np.array(msg.data)
        
        # Send command to hardware
        try:
            # Convert radians to degrees (orca_core uses degrees)
            angles_deg = np.degrees(self._last_command)
            
            # Build joint position dictionary
            # Assumes command array matches self.hand.joint_ids order
            joint_dict = {
                joint_name: float(angles_deg[i]) 
                for i, joint_name in enumerate(self.hand.joint_ids)
            }
            
            # Send to hardware
            self.hand.set_joint_pos(joint_dict)
            
        except Exception as e:
            self.get_logger().error(f'Failed to send command to hardware: {e}')
    
    def publish_joint_states(self):
        """
        Timer callback to publish current joint states at control frequency.
        Reads from hardware and publishes to ROS topic.
        """
        if not self.connected:
            return
        
        try:
            # Read current joint positions from hardware. Request as dict to
            # ensure .get() is available. OrcaHand.get_joint_pos may optionally
            # return a list when called with as_list=True; call with
            # as_list=False to get a dict. Be defensive and handle lists too.
            joint_pos = self.hand.get_joint_pos(as_list=False)

            # If for some reason a list was returned, convert to dict
            if isinstance(joint_pos, list):
                joint_pos_dict = {name: val for name, val in zip(self.hand.joint_ids, joint_pos)}
            else:
                joint_pos_dict = joint_pos

            # Convert to radians for ROS
            joint_angles_rad = [
                np.radians(joint_pos_dict.get(name, 0.0))
                for name in self.hand.joint_ids
            ]
            
            # Create JointState message
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.hand.joint_ids
            joint_state_msg.position = joint_angles_rad
            
            # Publish
            self.joint_state_pub.publish(joint_state_msg)
            
            # Store for local access
            with self._lock:
                self._current_joint_states = joint_angles_rad
                
        except Exception as e:
            self.get_logger().error(f'Failed to read/publish joint states: {e}')
    
    def shutdown(self):
        """Clean shutdown of hardware connection."""
        self.get_logger().info('Shutting down ORCA Hardware Node...')
        
        try:
            if self.connected:
                # Move to safe home position
                self.get_logger().info('Moving to home position...')
                self.hand.home()
                
                # Disable torque
                self.get_logger().info('Disabling torque...')
                self.hand.disable_torque()
                
                # Disconnect
                self.get_logger().info('Disconnecting...')
                self.hand.disconnect()
                
            self.get_logger().info('✓ Shutdown complete')
            
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')


def main(args=None):
    """Main entry point for the ROS 2 node."""
    rclpy.init(args=args)
    
    node = None
    try:
        node = OrcaHardwareNode()
        
        # Spin the node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\nKeyboard interrupt detected')
        
    except Exception as e:
        print(f'ERROR: {e}')
        
    finally:
        # Cleanup
        if node is not None:
            node.shutdown()
            node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
