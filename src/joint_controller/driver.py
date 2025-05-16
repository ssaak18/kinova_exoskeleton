#!/usr/bin/env python3
import rospy
import math
from dynamixel_sdk import * 
from pynput import keyboard
import sys
import numpy as np
from sensor_msgs.msg import JointState
import time

class InputDriver:

    old_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    def __init__(self, node_name):
        self.time = None
        self.frequency = None
        self.joint_input_pub = rospy.Publisher("/input_states", JointState, queue_size=1)
        self.is_publishing = True
        
    def make_joint_msg(self, pos, vel):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = [f"joint_{i+1}" for i in range(len(pos))]
        joint_state_msg.position = pos
        joint_state_msg.velocity = vel
        return joint_state_msg

    def start(self):
        """Start the driver and continuously publish joint positions."""
        rospy.loginfo(f"Starting {self.__class__.__name__}")
        rate = rospy.Rate(100)  # 10 Hz
        while not rospy.is_shutdown():
            self.get_joint_pos()
            rate.sleep()

class TestDriver(InputDriver):
    """Simulation driver"""

    def __init__(self):
        super().__init__("test_driver")

    def get_joint_pos(self):
        joint_pos = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        if self.is_publishing:
            self.joint_input_pub.publish(super().make_joint_msg(joint_pos, joint_vel))

class KeyDriver(InputDriver):
    """Keyboard driver"""
    
    global joint_pos 
    joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def __init__(self):
        super().__init__("key_driver")

        # Keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        
    def on_key_press(self, key):
        """Basic keyboard controls; keys above/below each other (ex. q/a) control +/-.1 for the 7 joints """
        try:
            if key.char == 'q':
                joint_pos[0] += .1
            elif key.char == 'a':
                joint_pos[0] -= .1
            elif key.char == 'w':
                joint_pos[1] += .1
            elif key.char == 's':
                joint_pos[1] -= .1
            elif key.char == 'e':
                joint_pos[2] += .1
            elif key.char == 'd':
                joint_pos[2] -= .1
            elif key.char == 'r':
                joint_pos[3] += .1
            elif key.char == 'f':
                joint_pos[3] -= .1
            elif key.char == 't':
                joint_pos[4] += .1
            elif key.char == 'g':
                joint_pos[4] -= .1
            elif key.char == 'y':
                joint_pos[5] += .1
            elif key.char == 'h':
                joint_pos[5] -= .1
            elif key.char == 'u':
                joint_pos[6] += .1
            elif key.char == 'j':
                joint_pos[6] -= .1
        except AttributeError:
            pass

    def get_joint_pos(self):
        if self.is_publishing:
            self.joint_input_pub.publish(super().make_joint_msg(joint_pos, self.joint_vel))

class ExoDriver(InputDriver):
    """Driver for controlling real exoskeleton hardware via Dynamixel motors"""
    zero_offsets = [0, 0, 0, 0, 0, 0, 0]


    PORT = "/dev/ttyUSB0"
    BAUDRATE = 4000000  # set for all servos
    PROTOCOL_VERSION = 2.0
    PRESENT_VEL_ADDR = 128
    PRESENT_POS_ADDR = 132  # Position address in control table (0-4095)
    MAX_POS = 4095.0

    

    def __init__(self, motor_ids, rev=False):
        super().__init__("exo_driver")
        self.motor_ids = motor_ids
        self.portHandler = PortHandler(self.PORT)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        self.rev = rev

        self.offsets = [-0.755, 2.377, -0.839, -2.310, -4.047, -5.49, -2.27]

        # Open port
        if self.portHandler.openPort() and self.portHandler.setBaudRate(self.BAUDRATE):
            rospy.loginfo("Connected to Dynamixel")
        else:
            rospy.logerr("Failed to connect to Dynamixel")
            exit()

    def get_joint_pos(self):
        joint_pos = []
        joint_vel = []

        for motor_id in self.motor_ids:
            dxl_pos, dxl_comm_result1, _ = self.packetHandler.read4ByteTxRx(self.portHandler, motor_id, self.PRESENT_POS_ADDR)
            dxl_vel, dxl_comm_result2, _ = self.packetHandler.read4ByteTxRx(self.portHandler, motor_id, self.PRESENT_VEL_ADDR)

            if dxl_comm_result1 != COMM_SUCCESS or dxl_comm_result2 != COMM_SUCCESS:
                rospy.logerr(f"Motor {motor_id}: Read error {dxl_comm_result1, dxl_comm_result2}")
                joint_pos.append(self.old_pos[motor_id - 1])
                continue

            dxl_pos = self.unsigned_to_signed(dxl_pos)

            if motor_id % 2 == 1:
                dxl_pos = -1 * dxl_pos

            pos_rad = (dxl_pos / self.MAX_POS * 2 * math.pi) % (np.pi * 2)
            pos_rad += self.offsets[motor_id - 1]

            # motor 2 must be switched for exo skeleton
            if motor_id == 2:
                pos_rad = -1 * (pos_rad + (np.pi / 2))

            if self.rev and motor_id % 2 == 0:
                pos_rad = -1 * pos_rad
            
            pos_rad = pos_rad % (np.pi * 2)
            if pos_rad < 0:
                pos_rad += np.pi * 2
            joint_pos.append(pos_rad)
            joint_vel.append(self.unsigned_to_signed(dxl_vel))

        relative_joint_pos = [
            curr - zero for curr, zero in zip(joint_pos, self.zero_offsets)
        ]

        self.old_pos = joint_pos

        if self.is_publishing:
            if self.time is None:
                self.time = time.time()
            else:
                diff = time.time() - self.time
                self.time = time.time()
                if self.frequency is None:
                    self.frequency = 1 / diff
                else:
                    self.frequency = (self.frequency + (1 / diff)) /2
                # print("Frequency: ", self.frequency)
            self.joint_input_pub.publish(super().make_joint_msg(relative_joint_pos, joint_vel))

    def unsigned_to_signed(self, val, bits=32):
        if val >= 2**(bits - 1):
            val -= 2**bits
        return val

def start():
    rospy.init_node("driver_node", anonymous=True)
    mode = rospy.get_param("~input_mode")
    rev_param = rospy.get_param("~rev")
    rospy.loginfo(f"[DRIVER] mode param: {mode}")
    rospy.loginfo(f"[DRIVER] rev param: {rev_param}")

    if mode == "exo":
        motor_ids = [1,2,3,4,5,6,7] # set for servos in wizard
        rev = False
        if rev_param == "back":
            rev = True
        exo_driver = ExoDriver(motor_ids, rev)
        exo_driver.start()
    elif mode == "test":
        test_driver = TestDriver()
        test_driver.start()
    elif mode == "key":
        key_driver = KeyDriver()
        key_driver.start()

if __name__ == "__main__": 
    start()
