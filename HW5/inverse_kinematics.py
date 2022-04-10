from math import sin, pi, atan2, acos, sqrt


def inverse_kinematics(position):
    x = position[0]
    y = position[1]
    z = position[2]

    # Link lengths
    l1 = 0.1039
    l2 = 0.1581
    l3 = 0.150
    l4 = 0.150

    # theta1 can be obtained from: tan(theta1) = y/x
    theta1 = atan2(y, x)

    # theta2 can be obtained from: theta2 + alpha + beta1 + gamma = 90
    # alpha
    alpha = acos(l4/l2)

    # gamma
    length_pb = z - l1
    length_ab = sqrt(x**2 + y**2)
    gamma = atan2(length_pb, length_ab)

    # beta1
    length_ap = length_pb/sin(gamma)
    beta1 = get_angle_from_law_of_cosines(l2, length_ap, l3)

    theta2 = pi/2 - alpha - beta1 - gamma

    # theta3 can be obtained from: theta3 + ext_angle for triangle ACD = beta2
    ext_angle = pi/2 + alpha
    beta2 = get_angle_from_law_of_cosines(l3, l2, length_ap)
    theta3 = beta2 - ext_angle

    return [theta1, theta2, theta3]


def get_angle_from_law_of_cosines(a, b, c):
    num = a**2 + b**2 - c**2
    denom = 2*a*b

    return acos(num/denom)
