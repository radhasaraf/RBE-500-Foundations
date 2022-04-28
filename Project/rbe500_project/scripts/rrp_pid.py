#!/usr/bin/env python3

# imports
import rospy
from rrp_ik_client import get_rrp_ik
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from math import pi
import numpy as np

# Poses
pose1 = [0.0, 0.77, 0.34]
pose2 = [-0.345, 0.425, 0.24]
pose3 = [0.67, -0.245, 0.14]
pose4 = [0.77, 0.0, 0.39]

# Run for every pose in a loop  # later
# run client function to get desired joint states
# des_states = get_rrp_ik(*pose1)
des_states = get_rrp_ik(0.0, 0.77, 0.34)


# Control logic
class Controller:
    def __init__(self, control_entity, p=0.0, d=0.0, i=0.0, set_point=0.0, rospy_rate=0):
        self.control_entity = control_entity
        self.Kp = p
        self.Kd = d
        self.Ki = i
        self.dt = rospy_rate
        self.set_point = set_point  # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        error = self.set_point - current_value
        if self.control_entity == "orientation":
            if error > pi:
                error -= 2*pi
            if error < -pi:
                error += 2*pi

        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error) / self.dt
        I_term = self.Ki * error * self.dt

        self.previous_error = error

        return P_term + D_term + I_term

    def update_set_point(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def set_pid(self, p=0.0, i=0.0, d=0.0):
        self.Kp = p
        self.Ki = i
        self.Kd = d


# Publish input commands to rrp
class RRP_bot:
    def __init__(self, x, y, z):
        rospy.init_node("rrp_bot_move")
        rospy.loginfo("Press Ctrl + C to terminate")

        self.jnt1_topic = "/rrp/joint1_effort_controller/command"
        self.jnt2_topic = "/rrp/joint2_effort_controller/command"
        self.jnt3_topic = "/rrp/joint3_effort_controller/command"

        self.rate = rospy.Rate(100)

        self.pos_control = Controller("position")
        self.orient_control = Controller("orientation")

        # Get desired joint states
        self.des_states = get_rrp_ik(x, y, z)

        self.actual_states = []
        self.logging_counter = 0
        self.trajectory = list()

        # Subscribe to joint states
        rospy.Subscriber("/rrp/joint_states", JointState, self.subs_callback)

        try:
            print("Will try and run now!")
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            # np.savetxt(
            #     "trajectory.csv",
            #     np.array(self.trajectory),
            #     fmt="%f", delimiter=","
            # )
            pass

    def subs_callback(self, data):
        self.actual_states = data.position

    def move_to_orientation(self, jnt_idx, topic, kp, ki, kd):
        des_state = self.des_states[jnt_idx]

        print("Actual:", self.actual_states)
        act_state = self.actual_states[jnt_idx]

        self.orient_control.set_pid(kp, ki, kd)
        self.orient_control.update_set_point(des_state)

        while not rospy.is_shutdown():
            control_input = self.orient_control.update(act_state)
            jnt_pub = rospy.Publisher(topic, Float64, queue_size=10)
            jnt_pub.publish(control_input)
            if self.is_close("orientation", des_state, act_state):
                self.stop()

    def move_to_position(self, jnt_idx, topic, kp, ki, kd):
        des_state = self.des_states[jnt_idx]
        act_state = self.actual_states[jnt_idx]

        self.pos_control.set_pid(kp, ki, kd)
        self.pos_control.update_set_point(des_state)

        while not rospy.is_shutdown():
            control_input = self.orient_control.update(act_state)
            jnt_pub = rospy.Publisher(topic, Float64, queue_size=10)
            jnt_pub.publish(control_input)
            if self.is_close("orientation", des_state, act_state):
                self.stop()

    @staticmethod
    def stop(jnt_pub):
        jnt_pub.publish(0.0)

    @staticmethod
    def is_close(entity, desired, actual, orient_tol=0.15, pos_tol=0.002):
        error = abs(desired - actual)
        tol = orient_tol if entity == "orientation" else pos_tol
        return True if error < tol else False

    def run(self):
        from time import sleep
        sleep(3)
        self.move_to_orientation(0, self.jnt1_topic, 1, 2, 3)
        self.move_to_orientation(1, self.jnt2_topic, 3, 4, 6)
        self.move_to_position(2, self.jnt3_topic, 1, 1, 1)


if __name__ == "__main__":
    RRP_bot(0.0, 0.77, 0.34)
