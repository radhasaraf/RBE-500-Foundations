#!/usr/bin/env python3

import sys
import rospy
from rbe500_project.srv import rrpIK

def get_rrp_ik(x, y, z):
    rospy.wait_for_service('calc_rrp_ik')
    try:
        calc_rrp_ik = rospy.ServiceProxy('calc_rrp_ik', rrpIK)
        response = calc_rrp_ik(float(x), float(y), float(z))
        print("Requesting ik for end effector position (%s %s %s)" % (x, y, z))
        return response.theta1, response.theta2, response.d3
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y z]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = sys.argv[1]
        y = sys.argv[2]
        z = sys.argv[3]
    else:
        print(usage())
        sys.exit(1)
    print("Requesting ik for end effector position (%s %s %s)"%(x, y, z))
    print("joint states : %s %s %s"%(get_rrp_ik(x, y, z)))