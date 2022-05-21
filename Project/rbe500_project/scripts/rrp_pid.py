#!/usr/bin/env python3

from math import pi
from typing import List, Tuple

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from rrp_ik_client import get_rrp_ik

# Poses
pose1 = [0.0, 0.77, 0.34]
pose2 = [-0.345, 0.425, 0.24]
pose3 = [0.67, -0.245, 0.14]
pose4 = [0.77, 0.0, 0.39]
poses = [pose1, pose2, pose3, pose4]

ROSPY_RATE = 150


# Control logic
class Controller:
    def __init__(
            self,
            control_entity: str,
            p: float = 0.0,
            d: float = 0.0,
            i: float = 0.0,
            set_point: float = 0.0,
            ):
        self.control_entity = control_entity
        self.Kp = p
        self.Kd = d
        self.Ki = i
        self.dt = 1/ROSPY_RATE
        self.set_point = set_point  # reference (desired value)
        self.previous_error = 0
        self.P_term = 0.0
        self.I_term = 0.0
        self.D_term = 0.0

    def update(self, current_value: float) -> float:
        error = self.set_point - current_value
        if self.control_entity == "orientation":
            if error > pi:
                error -= 2*pi
            if error < -pi:
                error += 2*pi

        self.P_term = self.Kp * error
        self.D_term = self.Kd * (error - self.previous_error) / self.dt
        self.I_term += self.Ki * error * self.dt

        self.previous_error = error

        return self.P_term + self.D_term + self.I_term

    def update_set_point(self, set_point: float) -> None:
        self.set_point = set_point
        self.previous_error = 0

    def set_pid(self, p: float = 0.0, i: float = 0.0, d: float = 0.0) -> None:
        self.Kp = p
        self.Ki = i
        self.Kd = d


# Publish input commands to rrp
class RRP_bot:
    def __init__(self, positions: List[Tuple]):
        rospy.init_node("rrp_bot_move")
        rospy.loginfo("Press Ctrl + C to terminate")

        self.poses = positions
        self.des_states = None

        self.jnt1_topic = "/rrp/joint1_effort_controller/command"
        self.jnt2_topic = "/rrp/joint2_effort_controller/command"
        self.jnt3_topic = "/rrp/joint3_effort_controller/command"

        self.jnt_pub1 = rospy.Publisher(self.jnt1_topic, Float64, queue_size=10)
        self.jnt_pub2 = rospy.Publisher(self.jnt2_topic, Float64, queue_size=10)
        self.jnt_pub3 = rospy.Publisher(self.jnt3_topic, Float64, queue_size=10)

        self.pos_control3 = Controller("position")
        self.orient_control1 = Controller("orientation")
        self.orient_control2 = Controller("orientation")

        self.actual_states = []
        self.logging_counter = 0
        self.trajectory = list()

        # Subscribe to joint states
        rospy.Subscriber("/rrp/joint_states", JointState, self.subs_callback)

        try:
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

    def control(self, kp1, ki1, kd1, kp2, ki2, kd2, kp3, ki3, kd3):

        reached_1 = False
        reached_2 = False
        reached_3 = False

        self.orient_control1.set_pid(kp1, ki1, kd1)
        self.orient_control2.set_pid(kp2, ki2, kd2)
        self.pos_control3.set_pid(kp3, ki3, kd3)

        des_state1 = self.des_states[0]
        des_state2 = self.des_states[1]
        des_state3 = self.des_states[2]

        self.orient_control1.update_set_point(des_state1)
        self.orient_control2.update_set_point(des_state2)
        self.pos_control3.update_set_point(des_state3)

        rate = rospy.Rate(ROSPY_RATE)
        while not rospy.is_shutdown():
            act_state1 = self.actual_states[0]
            act_state2 = self.actual_states[1]
            act_state3 = self.actual_states[2]

            control_input1 = self.orient_control1.update(act_state1)
            control_input2 = self.orient_control2.update(act_state2)
            control_input3 = self.pos_control3.update(act_state3)

            self.jnt_pub1.publish(control_input1)
            self.jnt_pub2.publish(control_input2)
            self.jnt_pub3.publish(control_input3)

            if self.is_close("orientation", des_state1, act_state1):
                reached_1 = True
                self.stop(self.jnt_pub1)
            if self.is_close("orientation", des_state2, act_state2):
                reached_2 = True
                self.stop(self.jnt_pub2)
            if self.is_close("position", des_state3, act_state3):
                reached_3 = True
                self.stop(self.jnt_pub3)

            if reached_1 and reached_2 and reached_3:
                print("Reached first pose!")
                break

            rate.sleep()

    @staticmethod
    def stop(jnt_pub):
        jnt_pub.publish(0.0)
        jnt_pub.publish(0.0)
        jnt_pub.publish(0.0)
        jnt_pub.publish(0.0)

    @staticmethod
    def is_close(entity, desired, actual, orient_tol=0.15, pos_tol=0.002):
        error = abs(desired - actual)
        tol = orient_tol if entity == "orientation" else pos_tol
        return True if error < tol else False

    def run(self):
        from time import sleep
        sleep(5)
        for pose in self.poses:
            x1, y1, z1 = pose

            # Get desired joint states
            self.des_states = get_rrp_ik(x1, y1, z1)

            self.control(0.25, 0.02, 0.45, 0.25, 0.0125, 0.1, 0.25, 0.01, 0.1)


if __name__ == "__main__":
    RRP_bot(
        [(0.0, 0.77, 0.34), (-0.345, 0.425, 0.24)]
    )
    

