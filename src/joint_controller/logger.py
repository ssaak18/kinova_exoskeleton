#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

class LoggerNode():
    def __init__(self):
        rospy.init_node("logger_node", anonymous=True)
        self.joint_input_sub = rospy.Subscriber("/input_states", JointState, self.joint_input_callback)
    
    def joint_input_callback(self, joint_state_msg):
        pos = f"Posistion: {', '.join(map(str, joint_state_msg.posistion))}"
        vel = f"Velocity: {', '.join(map(str, joint_state_msg.velocity))}"
        rospy.loginfo(pos)
        rospy.loginfo(vel)
        
    def camera_logger(self, camera_info):
        rospy.loginfo(camera_info)
    
    def skin_logger(self, skin_info):
        rospy.loginfo(skin_info)

if __name__ == "__main__":
    logger_node = LoggerNode()
    logger_node.start()

