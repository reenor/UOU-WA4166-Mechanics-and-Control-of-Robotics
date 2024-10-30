import numpy as np
from scipy.optimize import minimize

# Jacobian function for a 3-DOF manipulator
def jacobian(theta, ll):
    J = np.array(
        [
          [ -ll[0] * np.sin(theta[0]) - ll[1] * np.sin(theta[0] + theta[1]) - \
                                                    ll[2] * np.sin(theta[0] + theta[1] + theta[2]),
            -ll[1] * np.sin(theta[0] + theta[1]) - ll[2] * np.sin(theta[0] + theta[1] + theta[2]),
            -ll[2] * np.sin(theta[0] + theta[1] + theta[2])
          ],

          [ ll[0] * np.cos(theta[0]) + ll[1] * np.cos(theta[0] + theta[1]) + \
                                                    ll[2] * np.cos(theta[0] + theta[1] + theta[2]),
            ll[1] * np.cos(theta[0] + theta[1]) + ll[2] * np.cos(theta[0] + theta[1] + theta[2]),
            ll[2] * np.cos(theta[0] + theta[1] + theta[2])
          ]
        ])

    return J

# Singularity penalty function
def singularity_avoidance(theta, ll):
    J = jacobian(theta, ll)
    rank_J = np.linalg.matrix_rank(J)
    if rank_J < 2:
        return 1e6  # Large penalty for singular configurations

    return 1 / (np.linalg.norm(J, ord=2)**2)  # Use the Frobenius norm to avoid singularity issues

# Objective function for optimization
def objective(theta, target_position, ll, alpha, beta):

    # Forward kinematics to calculate end effector position
    x = ll[0] * np.cos(theta[0]) + ll[1] * np.cos(theta[0] + theta[1]) + \
                                                    ll[2] * np.cos(theta[0] + theta[1] + theta[2])
    y = ll[0] * np.sin(theta[0]) + ll[1] * np.sin(theta[0] + theta[1]) + \
                                                    ll[2] * np.sin(theta[0] + theta[1] + theta[2])

    position_error = np.linalg.norm(np.array([x, y]) - target_position)
    singularity_penalty = singularity_avoidance(theta, ll)

    return alpha * position_error + beta * singularity_penalty

# Link lengths
ll = [1.0, 1.0, 1.0]

# Desired end effector position
target_position = np.array([0.6, 2.4])

# Start with an initial guess for joint angles: theta1 = theta2 = theta3 = 30 degrees
initial_theta = np.radians([30, 30, 30])

alpha = 1.0
beta = 0.5

# Optimize joint angles
result = minimize(objective, initial_theta, args=(target_position, ll, alpha, beta))
optimized_theta = result.x

# Show results
print("Optimized joint angles 1:", np.degrees(optimized_theta[0]))
print("Optimized joint angles 2:", np.degrees(optimized_theta[1]))
print("Optimized joint angles 3:", np.degrees(optimized_theta[2]))
