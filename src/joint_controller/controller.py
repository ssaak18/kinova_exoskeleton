#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from pynput import keyboard
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import KinovaCommand, JointTrajectoryCommand, JointCommand, JointVelocityCommand, CartesianCommand, OpenGripperCommand, CloseGripperCommand
from scipy import signal
import atexit


EXO_VAL_MAX = 100 # out of the dynamixel range (need to confirm)
ROBO_VEL_MAX = .7
JOINT_POS_ERROR = .1 # rad

VEL_MULTIPLIER = .5

class SafetySwitchControlled:
    
    old_pos = None
    safe_speed = True
    
    def __init__(self, node_name):
        
        self.joint_input_sub = rospy.Subscriber("/input_states", JointState, self.joint_input_callback)
        self.kinova_joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

        self.good_to_start = False
        self.send = False

        self.filter_pos = np.array(0.0 * 7)
        self.prev_error = np.array(0.0 * 7)

        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

        self.zeroing = False

    def on_key_press(self, key):
        try:
            if key.char == 'n':
                print("Control Activated!")
                self.safe_speed = True
                self.send = not self.send
        except AttributeError:
            pass

    def joint_input_callback(self, joint_state_msg):
        if self.old_pos is None:
            self.old_pos = joint_state_msg.position

        safe_move = self.safe_move(joint_state_msg.velocity, joint_state_msg.position)
            
        in_bounds_pos = safe_move["pos"]
        new_joint_state_msg = JointState()
        new_joint_state_msg.header.stamp = rospy.Time.now()
        new_joint_state_msg.name = [f"joint_{i+1}" for i in range(len(in_bounds_pos))]
        new_joint_state_msg.position = in_bounds_pos
        new_joint_state_msg.velocity = safe_move["vel"]
        # self.kinova_joint_state_pub.publish(new_joint_state_msg)
        
    def safe_move(self, joint_vel, new_pos):
        vel = np.array(joint_vel)
        robo_state = self.arm.get_state()
        robo_vel = robo_state["velocity"] 
        robo_pos = (robo_state["position"] + (np.pi * 2)) % (np.pi * 2)

        self.send_vel = np.zeros(7)

        self.safe_speed = self.safe_speed and np.all((vel >= -1 * EXO_VAL_MAX) & (vel <= EXO_VAL_MAX)) and np.all((robo_vel >= -1 * ROBO_VEL_MAX) & (robo_vel <= ROBO_VEL_MAX))
        
        if self.safe_speed:
            pos_diff = np.asarray((new_pos) - robo_pos)

            pos_diff[pos_diff >= np.pi] -= 2 * np.pi
            pos_diff[pos_diff < -np.pi] += 2 * np.pi

            kp = 5.4
            kd = 0

            error_d = (pos_diff - self.prev_error) / .025
            self.prev_error = pos_diff

            velocity = kp * pos_diff - kd * error_d

            norm = np.linalg.norm(velocity)

            if norm < .01:
                self.send_vel = np.zeros(7)
            elif norm > VEL_MULTIPLIER:
                self.send_vel = (velocity / norm) * VEL_MULTIPLIER
            else:
                self.send_vel = velocity

            self.old_pos = new_pos
        else:
            self.arm.execute_command(JointVelocityCommand([0]*7))
            input("Sudden movement detected! Press enter to reset.")
            self.safe_speed = True
            self.good_to_start = not self.good_to_start
            
        return {"pos": self.old_pos, "vel": self.send_vel}

    def check_joint_limit(self, joint_pos):
        """ Ensures joint values remain in Kinova joint ranges """
        max = float('inf') 
        min = -float('inf') 
        check_pos = np.array(joint_pos)
        joint_min = np.array([min, -2.25, min, -2.58, min, -2.1, min])
        joint_max = np.array([max, 2.25, max, 2.58, max, 2.1, max])
        return np.clip(check_pos, joint_min, joint_max)

    def start(self):
        rospy.loginfo(f"Starting {self.__class__.__name__}")
        rospy.spin()

    def exit(self):
        print("Goodbye")

class KinovaController(SafetySwitchControlled):

    def __init__(self):
        self.arm = ArmInterfaceClient()
        super().__init__("kinova_controller")

        self.kinova_joint_state_sub = rospy.Subscriber("/robot_joint_states", JointState, self.kinova_state_callback)
        self.current_state = self.arm.get_state()["position"]

    def kinova_state_callback(self, kinova_state_msg):
        self.current_state = kinova_state_msg.position

    def check_pos(self, joint_state_msg):
        current_state = self.arm.get_state()["position"]
        current_state = (np.asarray(current_state) + (np.pi * 2)) % (np.pi * 2)

        """Checks if the exo and robot are in the initial position before starting."""
        if not self.good_to_start:
            self.send = False

            init_pos = np.array([0.0] * 7)
            robot_zero = [0, 0, 0, 0, 0, 0, 0]
            exo_joint = np.asarray(joint_state_msg.position) % (np.pi * 2)
            diff = np.asarray(exo_joint - init_pos)
            diff[diff > np.pi] -= (np.pi * 2)
            
            exo_in = np.all(np.abs(diff) <= JOINT_POS_ERROR)
            
            kinova_diff = current_state - init_pos
            kinova_diff[kinova_diff > np.pi] -= (np.pi * 2)

            kinova_in = np.all(np.abs(kinova_diff) <= JOINT_POS_ERROR)

            if not kinova_in:
                input("Press enter to zero arm.")
                while not np.all(np.abs(init_pos - self.arm.get_state()["position"]) <= JOINT_POS_ERROR):
                    self.arm.execute_command(JointCommand(robot_zero))

            print("Exoskeleton Error:", np.round(np.abs(diff), 3))

            self.good_to_start = exo_in and kinova_in

            if self.good_to_start:
                print("Initial position reached! Press 'n' to unfreeze robot.")

    def joint_input_callback(self, input_state_msg):
        self.check_pos(input_state_msg)
        """Publishes joint states only if the robot is ready to start."""
        joint_positions = np.arange(7)
        if self.good_to_start and self.send:
            super().joint_input_callback(input_state_msg)
            deg_vel = np.degrees(self.send_vel)
            self.arm.execute_command(JointVelocityCommand(deg_vel))
    
    def exit(self):
        self.arm.execute_command(JointVelocityCommand([0]*7))

    
class SimController(SafetySwitchControlled):
    def __init__(self):
        super().__init__("sim_controller")

def on_exit(controller):
    controller.exit()

if __name__ == "__main__": 
    rospy.init_node("controller_node", anonymous=True)
    mode = rospy.get_param("~robot_mode")
    rospy.loginfo(f"[CONTROLLER] mode param: {mode}")

    if mode == "sim":
        controller = SimController()
    elif mode == "kinova":
        controller = KinovaController()
    
    atexit.register(lambda: on_exit(controller))

    controller.start()
