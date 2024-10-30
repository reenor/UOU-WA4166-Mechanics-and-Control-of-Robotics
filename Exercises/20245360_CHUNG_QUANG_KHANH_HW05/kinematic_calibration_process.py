
import numpy as np
from scipy.optimize import minimize

def forward_kinematics(l, theta):
    x = l[0] * np.cos(theta[0]) + l[1] * np.cos(theta[0] + theta[1]) + l[2] * np.cos(theta[0] + theta[1] + theta[2])
    y = l[0] * np.sin(theta[0]) + l[1] * np.sin(theta[0] + theta[1]) + l[2] * np.sin(theta[0] + theta[1] + theta[2])

    return np.array([x, y])

def error_function(l, theta, x_measured, y_measured):
    x_computed, y_computed = forward_kinematics(l, theta)

    return (x_measured - x_computed) ** 2 + (y_measured - y_computed) ** 2

# Choose the configuration with theta1 = 30, theta2 = 45, theta3 = 60 degrees
theta = [np.deg2rad(30), np.deg2rad(45), np.deg2rad(60)]

# Real link lengths
l_real = [0.95, 0.95, 0.95]

# Compute the actual position and consider it as the measured position
x_measured, y_measured = forward_kinematics(l_real, theta)

# Start with an initial guess for link lengths
initial_guess = [1, 1, 1]

# Optimization
result = minimize(error_function, initial_guess, args=(theta, x_measured, y_measured))
optimized_lengths = result.x

# Show results
print("Optimized link length 1:", optimized_lengths[0])
print("Optimized link length 2:", optimized_lengths[1])
print("Optimized link length 3:", optimized_lengths[2])
