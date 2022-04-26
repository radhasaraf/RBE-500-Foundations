#!/usr/bin/env python3

from math import pi, sqrt

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

MIN_LINEAR_VEL = 0.25
path = {
    (4, 0): {"pos_set_pt_idx": 0, "orient_set_pt": 0.0},
    (4, 4): {"pos_set_pt_idx": 1, "orient_set_pt": pi/2},
    (0, 4): {"pos_set_pt_idx": 0, "orient_set_pt": pi},
    (0, 0): {"pos_set_pt_idx": 1, "orient_set_pt": -pi/2},
}


class Turtlebot:
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel = Twist()
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("/reset", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()

        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.pos_control = Controller("position")
        self.orient_control = Controller("orientation")

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt(
                "trajectory.csv",
                np.array(self.trajectory),
                fmt="%f", delimiter=","
            )

    def run(self):
        self.move_to_waypoint((4, 0), pos_kp=0.05, orient_kp=0.25, orient_kd=0.8)
        self.stop()
        self.move_to_orientation(pi/2)
        self.stop()
        self.move_to_waypoint((4, 4), pos_kp=0.05, orient_kp=1, orient_kd=25)
        self.stop()
        self.move_to_orientation(pi)
        self.stop()
        self.move_to_waypoint((0, 4), pos_kp=0.05, orient_kp=1, orient_kd=25)
        self.stop()
        self.move_to_orientation(-pi/2)
        self.stop()
        self.move_to_waypoint((0, 0), pos_kp=0.05, orient_kp=1, orient_kd=25)
        self.stop()

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y]) # save trajectory
            rospy.loginfo(
                "odom: x="
                + str(self.pose.x)
                + "; y="
                + str(self.pose.y)
                + "; theta=" + str(yaw)
            )

    def move_to_waypoint(self, waypoint, pos_kp, orient_kp, orient_kd):
        self.pos_control.set_pd(pos_kp, 0)
        index_of_interest = path[waypoint]["pos_set_pt_idx"]
        self.pos_control.update_set_point(waypoint[index_of_interest])

        self.orient_control.set_pd(orient_kp, orient_kd)
        self.orient_control.update_set_point(path[waypoint]["orient_set_pt"])

        while not rospy.is_shutdown():
            ang_vel_z = self.orient_control.update(self.pose.theta)

            lin_vel_x = self.pos_control.update(self.pose.x)
            if lin_vel_x < MIN_LINEAR_VEL:
                lin_vel_x = MIN_LINEAR_VEL

            self.vel.linear.x = lin_vel_x
            self.vel.angular.z = ang_vel_z

            self.vel_pub.publish(self.vel)

            if self.in_close_proximity_pos(waypoint, tol=0.1):
                self.stop()
                break

    def move_to_orientation(self, theta_des):
        self.orient_control.set_pd(2, 50)
        self.orient_control.update_set_point(theta_des)

        while not rospy.is_shutdown():
            self.vel.angular.z = self.orient_control.update(self.pose.theta)
            self.vel_pub.publish(self.vel)

            if self.in_close_proximity_ang(theta_des, tol=0.005):
                self.stop()
                break

    def stop(self):
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)

    def in_close_proximity_pos(self, waypoint, tol=0.125):
        dist_error = sqrt((self.pose.x-waypoint[0])**2 + (self.pose.y-waypoint[1])**2)
        return True if dist_error < tol else False

    def in_close_proximity_ang(self, angle, tol=0.01):
        angle_error = abs(self.pose.theta - angle)
        return True if angle_error < tol else False


class Controller:
    def __init__(self, control_entity, p=0.0, d=0.0, set_point=0.0):
        self.control_entity = control_entity
        self.Kp = p
        self.Kd = d
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
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term

    def update_set_point(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def set_pd(self, p=0.0, d=0.0):
        self.Kp = p
        self.Kd = d


if __name__ == "__main__":
    Turtlebot()
