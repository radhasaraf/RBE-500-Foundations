#!/usr/bin/env python3

from math import cos, acos, atan2, sqrt

import rospy
from rbe500_project.srv import rrpIK

# RRP bot constants
l1 = 0.425
l2 = 0.345


def get_rrp_ik(request):
	x=request.x
	y=request.y
	z=request.z

	num = x**2 + y**2 - l1**2 - l2**2
	denom = 2 * l1 * l2
	cos_term = num/denom
	sin_term = sqrt(1 - cos_term ** 2)
	theta_2 = atan2(sin_term, cos_term)

	alpha = atan2(y, x)
	num_ = l1 + l2 * cos(theta_2)
	denom_ = sqrt(x**2 + y**2)

	cos_term_ = num_/denom_
	sin_term_ = sqrt(1 - cos_term_ ** 2)
	beta = atan2(sin_term_, cos_term_)

	# beta = acos(num_/denom_)
	theta_1 = alpha - beta

	d3 = 0.39 - z
	return theta_1, theta_2, d3


def rrp_ik_server():
	rospy.init_node("rrp_ik_server")
	_ = rospy.Service("calc_rrp_ik", rrpIK, get_rrp_ik)
	rospy.spin()


if __name__ == '__main__':
	rrp_ik_server()
