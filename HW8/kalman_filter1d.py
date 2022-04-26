import numpy as np

# Random walk
# xk = xk-1 + R
# zk = xk + Q

A = [1]  # State matrix
B = [0]  # Input matrix
C = [1]  # Output matrix
I = [1]  # Identity

R = [4]  # system_noise
Q = [1]  # sensor_noise

u = [0]  # Control input
z1 = [22]
z2 = [23]

def kalman_filter(prev_mean, prev_var, control_action, sensor_measure):
    mew_bar = np.dot(A, prev_mean) + np.dot(B, control_action)
    sigma_bar = np.dot(np.dot(A, prev_var), np.transpose(A)) + R

    i1 = np.dot(C, sigma_bar)
    i2 = np.dot(i1, np.transpose(C))
    i3 = 1/(i2 + Q)  # np.linalg.inv(a2 + Q)
    innovation = np.dot(np.dot(sigma_bar, np.transpose(C)), i3)

    mew = mew_bar + np.dot(innovation, (sensor_measure - np.dot(C, mew_bar)))
    sigma = (I - innovation * C) * sigma_bar

    return mew, sigma

if __name__ == "__main__":
    prev_state_mean = 20
    prev_state_var = 9
    print(f"(Previous) mean, variance: {prev_state_mean}, {prev_state_var}")
    mean, var = kalman_filter([prev_state_mean], [prev_state_var], u, z1)
    print(f"(Z=22) mean, variance: {mean}, {var[0]}")
    mean, var = kalman_filter([mean], var, u, z2)
    print(f"(Z=23) mean, variance: {mean}, {var[0]}")
