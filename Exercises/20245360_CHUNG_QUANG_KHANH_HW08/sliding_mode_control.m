%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Student name: CHUNG QUANG KHANH
% Student ID:   20245360
% Homework:     08
% Professor:    KANG

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc, clearvars, close all

% Given boundary conditions for joint 1 (theta_1)
theta_1_0 = 30;  % Initial position (degrees)
theta_1_f = 150; % Final position (degrees)
theta_1_dot_0 = 0; % Initial velocity
theta_1_dot_f = 0; % Final velocity

% Given boundary conditions for joint 2 (theta_2)
theta_2_0 = 120; % Initial position (degrees)
theta_2_f = 30;  % Final position (degrees)
theta_2_dot_0 = 0; % Initial velocity
theta_2_dot_f = 0; % Final velocity

% Final time
t_f = 2; % seconds

% Mass parameters
m1 = 5; % real mass of joint 1 (kg)
m2 = 5; % real mass of joint 2 (kg)
m1_hat = 7; % known mass of joint 1 (kg)
m2_hat = 3; % known mass of joint 2 (kg)

% Link lengths
l1 = 0.5; % Length of link 1 (m)
l2 = 0.5; % Length of link 2 (m)

% Gravity constant
g = 9.81; % m/s^2

% Sliding Mode Controller Gains
lambda = 5; % Sliding mode gain for tracking
K = 10;     % Sliding mode control gain
K1 = 3;     % Additional gain for stability

% Solve for the coefficients of the cubic polynomial for joint 1 (theta_1)
A1 = [0^3, 0^2, 0, 1;
      t_f^3, t_f^2, t_f, 1;
      3*t_f^2, 2*t_f, 1, 0;
      6*t_f, 2, 0, 0];

b1 = [theta_1_0; theta_1_f; theta_1_dot_0; theta_1_dot_f];

coeffs_1 = A1 \ b1;  % Solve for the coefficients

% Solve for the coefficients of the cubic polynomial for joint 2 (theta_2)
A2 = [0^3, 0^2, 0, 1;
      t_f^3, t_f^2, t_f, 1;
      3*t_f^2, 2*t_f, 1, 0;
      6*t_f, 2, 0, 0];

b2 = [theta_2_0; theta_2_f; theta_2_dot_0; theta_2_dot_f];

coeffs_2 = A2 \ b2;  % Solve for the coefficients

% Time vector for simulation
t = linspace(0, t_f, 100);
theta_1_d = coeffs_1(1)*t.^3 + coeffs_1(2)*t.^2 + coeffs_1(3)*t + coeffs_1(4);
theta_2_d = coeffs_2(1)*t.^3 + coeffs_2(2)*t.^2 + coeffs_2(3)*t + coeffs_2(4);

% Initialize state variables
theta_1 = theta_1_0; theta_2 = theta_2_0;  % initial positions
theta_1_dot = theta_1_dot_0; theta_2_dot = theta_2_dot_0;  % initial velocities

% Sliding mode controller simulation loop
theta_1_actual = zeros(size(t));
theta_2_actual = zeros(size(t));
theta_1_vel = zeros(size(t));
theta_2_vel = zeros(size(t));
error_1 = zeros(size(t)); % Error for joint 1
error_2 = zeros(size(t)); % Error for joint 2

for i = 1:length(t)
    % Compute the sliding variable for joint 1 and joint 2
    s1 = theta_1_dot + lambda * (theta_1 - theta_1_d(i));
    s2 = theta_2_dot + lambda * (theta_2 - theta_2_d(i));
    
    % Compute the control input for joint 1 and joint 2 using the sliding mode control law
    tau_1 = - K * sign(s1) - K1 * s1;
    tau_2 = - K * sign(s2) - K1 * s2;
    
    % Compute the joint accelerations (simplified model)
    % In this model, we assume that the joint accelerations are given by the torques and the inertia matrix.
    
    % Inertia matrix for each joint
    I1 = m1 * l1^2 + m2 * (l1^2 + l2^2 + 2*l1*l2*cos(theta_2));
    I2 = m2 * l2^2;
    
    % Simplified dynamic equations (ignoring Coriolis and gravity for simplicity)
    theta_1_ddot = tau_1 / I1;
    theta_2_ddot = tau_2 / I2;
    
    % Update joint velocities and positions using numerical integration (Euler method)
    theta_1_dot = theta_1_dot + theta_1_ddot * (t(2) - t(1));
    theta_2_dot = theta_2_dot + theta_2_ddot * (t(2) - t(1));
    
    theta_1 = theta_1 + theta_1_dot * (t(2) - t(1));
    theta_2 = theta_2 + theta_2_dot * (t(2) - t(1));
    
    % Store the actual joint positions and velocities
    theta_1_actual(i) = theta_1;
    theta_2_actual(i) = theta_2;
    theta_1_vel(i) = theta_1_dot;
    theta_2_vel(i) = theta_2_dot;
    
    % Compute position errors
    error_1(i) = theta_1_d(i) - theta_1;
    error_2(i) = theta_2_d(i) - theta_2;
end

% Plot results
figure;

% Plot Joint 1 Actual and Desired Trajectories with Error
subplot(3,1,1);
plot(t, theta_1_actual, 'r-', 'LineWidth', 2);
hold on;
plot(t, theta_1_d, 'b--', 'LineWidth', 2);
title('Joint 1 Trajectory (\theta_1)');
xlabel('Time (s)');
ylabel('Angle (\theta_1) [degrees]');
legend('Actual', 'Desired');
grid on;

% Plot Joint 2 Actual and Desired Trajectories with Error
subplot(3,1,2);
plot(t, theta_2_actual, 'r-', 'LineWidth', 2);
hold on;
plot(t, theta_2_d, 'b--', 'LineWidth', 2);
title('Joint 2 Trajectory (\theta_2)');
xlabel('Time (s)');
ylabel('Angle (\theta_2) [degrees]');
legend('Actual', 'Desired');
grid on;

% Plot Position Errors
subplot(3,1,3);
plot(t, error_1, 'r-', 'LineWidth', 2);
hold on;
plot(t, error_2, 'b-', 'LineWidth', 2);
title('Position Errors');
xlabel('Time (s)');
ylabel('Error [degrees]');
legend('Error \theta_1', 'Error \theta_2');
grid on;
