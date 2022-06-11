#!/usr/bin/env python3

from time import sleep
from typing import List, Tuple

import numpy as np
import rospy
from rrp_ik_client import get_rrp_ik
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

ROSPY_RATE = 150


# Control logic
class Controller:
    def __init__(
        self, p: float = 0.0, i: float = 0.0, d: float = 0.0, set_point: float = 0.0
    ) -> None:
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.set_point = set_point  # reference (desired value)
        self.previous_error = 0

        self.dt = 1 / ROSPY_RATE
        self.P_term = 0.0
        self.I_term = 0.0
        self.D_term = 0.0

    def update(self, current_value: float) -> float:
        error = self.set_point - current_value

        self.P_term = self.Kp * error
        self.D_term = self.Kd * (error - self.previous_error) / self.dt
        self.I_term += self.Ki * error * self.dt

        self.previous_error = error

        return self.P_term + self.D_term + self.I_term

    def update_set_point(self, set_point: float) -> None:
        self.set_point = set_point
        self.previous_error = 0
        self.I_term = 0

    def set_pid(self, p: float = 0.0, i: float = 0.0, d: float = 0.0) -> None:
        self.Kp = p
        self.Ki = i
        self.Kd = d


class RRP_bot:
    def __init__(self, positions: List[Tuple]) -> None:
        rospy.init_node("rrp_bot_move")
        rospy.loginfo("Press Ctrl + C to terminate")

        self.poses = positions
        self.des_states = None
        self.actual_states = []

        self.jnt_pub1 = rospy.Publisher(
            "/rrp/joint1_effort_controller/command", Float64, queue_size=10
        )
        self.jnt_pub2 = rospy.Publisher(
            "/rrp/joint2_effort_controller/command", Float64, queue_size=10
        )
        self.jnt_pub3 = rospy.Publisher(
            "/rrp/joint3_effort_controller/command", Float64, queue_size=10
        )

        self.orient_control1 = Controller()
        self.orient_control2 = Controller()
        self.pos_control3 = Controller()

        # Subscribe to joint states
        rospy.Subscriber("/rrp/joint_states", JointState, self.subs_callback)

        self.logging_counter = 0

        self.trajectory1 = list()
        self.trajectory2 = list()
        self.trajectory3 = list()

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save joint trajectories into csv file
            np.savetxt(
                "trajectory1.csv", np.array(self.trajectory1), fmt="%f", delimiter=","
            )
            np.savetxt(
                "trajectory2.csv", np.array(self.trajectory2), fmt="%f", delimiter=","
            )
            np.savetxt(
                "trajectory3.csv", np.array(self.trajectory3), fmt="%f", delimiter=","
            )

    def subs_callback(self, data):
        self.actual_states = data.position

        # logging once every 25 times
        self.logging_counter += 1
        if self.logging_counter == 25:
            self.logging_counter = 0
            self.trajectory1.append(self.actual_states[0])
            self.trajectory2.append(self.actual_states[1])
            self.trajectory3.append(self.actual_states[2])

    def control(self, kp1, ki1, kd1, kp2, ki2, kd2, kp3, ki3, kd3):

        self.orient_control1.set_pid(kp1, ki1, kd1)
        self.orient_control2.set_pid(kp2, ki2, kd2)
        self.pos_control3.set_pid(kp3, ki3, kd3)

        self.orient_control1.update_set_point(self.des_states[0])
        self.orient_control2.update_set_point(self.des_states[1])
        self.pos_control3.update_set_point(self.des_states[2])

        rate = rospy.Rate(ROSPY_RATE)
        while not rospy.is_shutdown():

            reached_1, reached_2, reached_3 = False, False, False

            control_input1 = self.orient_control1.update(self.actual_states[0])
            control_input2 = self.orient_control2.update(self.actual_states[1])
            control_input3 = self.pos_control3.update(self.actual_states[2])

            self.jnt_pub1.publish(control_input1)
            self.jnt_pub2.publish(control_input2)
            self.jnt_pub3.publish(control_input3)

            if self.is_close("orientation", self.des_states[0], self.actual_states[0]):
                reached_1 = True
            if self.is_close("orientation", self.des_states[1], self.actual_states[1]):
                reached_2 = True
            if self.is_close("position", self.des_states[2], self.actual_states[2]):
                reached_3 = True

            if reached_1 and reached_2 and reached_3:
                print("Reached pose!")
                self.jnt_pub1.publish(-control_input1)
                self.jnt_pub2.publish(-control_input2)
                self.jnt_pub3.publish(-control_input3)
                sleep(1)
                break

            rate.sleep()

    @staticmethod
    def is_close(entity, desired, actual, orient_tol=0.15, pos_tol=0.002):
        error = abs(desired - actual)
        tol = orient_tol if entity == "orientation" else pos_tol
        return True if error < tol else False

    def run(self):
        sleep(3)  # To allow subscriber sufficient time for setup
        for pose in self.poses:
            x1, y1, z1 = pose
            self.des_states = get_rrp_ik(x1, y1, z1)
            self.control(0.25, 0.02, 0.45, 0.25, 0.0125, 0.1, 0.25, 0.01, 0.1)


if __name__ == "__main__":
    RRP_bot(
        [
            (0.0, 0.77, 0.34),
            (-0.345, 0.425, 0.24),
            (0.67, -0.245, 0.14),
            (0.77, 0.0, 0.39)
        ]
    )
