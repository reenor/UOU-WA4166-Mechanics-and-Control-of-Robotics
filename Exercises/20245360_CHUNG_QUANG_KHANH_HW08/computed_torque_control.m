%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Student name: CHUNG QUANG KHANH
% Student ID:   20245360
% Homework:     08
% Professor:    KANG

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc, clearvars, close all

% Given data
theta_1_0 = 30;  % Initial position for joint 1 (degrees)
theta_1_f = 150; % Final position for joint 1 (degrees)
theta_2_0 = 120; % Initial position for joint 2 (degrees)
theta_2_f = 30;  % Final position for joint 2 (degrees)
theta_dot_1_0 = 0;  % Initial velocity for joint 1 (deg/s)
theta_dot_1_f = 0;  % Final velocity for joint 1 (deg/s)
theta_dot_2_0 = 0;  % Initial velocity for joint 2 (deg/s)
theta_dot_2_f = 0;  % Final velocity for joint 2 (deg/s)
t_f = 2;  % Final time (seconds)

% Robot parameters
l1 = 0.5;  % Length of link 1 (meters)
l2 = 0.5;  % Length of link 2 (meters)
m1 = 5;    % Mass of link 1 (kg)
m2 = 5;    % Mass of link 2 (kg)
g = 9.81;  % Gravitational acceleration (m/s^2)

% Boundary conditions for theta_1
A1 = [0, 0, 0, 1;         % theta_1(0) = theta_1_0
      0, 0, 1, 0;         % theta_dot_1(0) = theta_dot_1_0
      t_f^3, t_f^2, t_f, 1; % theta_1(t_f) = theta_1_f
      3*t_f^2, 2*t_f, 1, 0]; % theta_dot_1(t_f) = theta_dot_1_f

b1 = [theta_1_0; 
      theta_dot_1_0; 
      theta_1_f; 
      theta_dot_1_f];

% Solve for the coefficients of the cubic polynomial for joint 1
coeffs_1 = A1\b1;

% Boundary conditions for theta_2
A2 = [0, 0, 0, 1;         % theta_2(0) = theta_2_0
      0, 0, 1, 0;         % theta_dot_2(0) = theta_dot_2_0
      t_f^3, t_f^2, t_f, 1; % theta_2(t_f) = theta_2_f
      3*t_f^2, 2*t_f, 1, 0]; % theta_dot_2(t_f) = theta_dot_2_f

b2 = [theta_2_0; 
      theta_dot_2_0; 
      theta_2_f; 
      theta_dot_2_f];

% Solve for the coefficients of the cubic polynomial for joint 2
coeffs_2 = A2\b2;

% Time vector from 0 to t_f with 100 points
t = linspace(0, t_f, 100);

% Calculate desired joint angles theta_1(t) and theta_2(t)
theta_1_t = coeffs_1(1)*t.^3 + coeffs_1(2)*t.^2 + coeffs_1(3)*t + coeffs_1(4);
theta_2_t = coeffs_2(1)*t.^3 + coeffs_2(2)*t.^2 + coeffs_2(3)*t + coeffs_2(4);

% Compute joint velocities (1st derivative of theta)
theta_dot_1_t = 3*coeffs_1(1)*t.^2 + 2*coeffs_1(2)*t + coeffs_1(3);
theta_dot_2_t = 3*coeffs_2(1)*t.^2 + 2*coeffs_2(2)*t + coeffs_2(3);

% Compute joint accelerations (2nd derivative of theta)
theta_ddot_1_t = 6*coeffs_1(1)*t + 2*coeffs_1(2);
theta_ddot_2_t = 6*coeffs_2(1)*t + 2*coeffs_2(2);

% Preallocate arrays for torques and simulated joint positions
theta_1_actual = zeros(size(t));
theta_2_actual = zeros(size(t));
theta_dot_1_actual = zeros(size(t));
theta_dot_2_actual = zeros(size(t));

% Initial conditions for the joint angles and velocities
theta_1 = theta_1_0;
theta_2 = theta_2_0;
theta_dot_1 = theta_dot_1_0;
theta_dot_2 = theta_dot_2_0;

% Simulate the system using numerical integration
for i = 1:length(t)
    % Compute the desired torques using the computed torque control law
    M11 = m1*l1^2 + m2*(l1^2 + l2^2 + 2*l1*l2*cos(theta_2));
    M12 = m2*(l2^2 + l1*l2*cos(theta_2));
    M21 = M12;
    M22 = m2*l2^2;

    M = [M11, M12; M21, M22];

    V1 = -m2*l1*l2*sin(theta_2)*theta_dot_2;
    V2 = m2*l1*l2*sin(theta_2)*(theta_dot_1 + theta_dot_2);
    V = [V1; V2];

    G1 = (m1*g*l1 + m2*g*(l1*cos(theta_1) + l2*cos(theta_1 + theta_2)));
    G2 = m2*g*l2*cos(theta_1 + theta_2);
    G = [G1; G2];

    % Desired joint accelerations from the trajectory
    theta_ddot_1 = theta_ddot_1_t(i);
    theta_ddot_2 = theta_ddot_2_t(i);

    % Computed torque control: tau = M * ddot(theta) + V + G
    tau = M * [theta_ddot_1; theta_ddot_2] + V + G;

    % Numerical integration (Euler method) to update joint angles and velocities
    % Assuming small time steps (dt)
    dt = t(2) - t(1);  % Time step (constant)

    % Update the joint velocities and angles (simple Euler integration)
    theta_dot_1 = theta_dot_1 + tau(1) * dt;
    theta_dot_2 = theta_dot_2 + tau(2) * dt;

    theta_1 = theta_1 + theta_dot_1 * dt;
    theta_2 = theta_2 + theta_dot_2 * dt;

    % Store the actual joint angles and velocities
    theta_1_actual(i) = theta_1;
    theta_2_actual(i) = theta_2;
    theta_dot_1_actual(i) = theta_dot_1;
    theta_dot_2_actual(i) = theta_dot_2;
end

% Compute position errors (desired - actual)
error_1 = theta_1_t - theta_1_actual;
error_2 = theta_2_t - theta_2_actual;

% Plot the position errors for both joints
figure;

subplot(2,1,1);
plot(t, error_1, 'r', 'LineWidth', 2);
title('Position Error for Joint 1');
xlabel('Time (seconds)');
ylabel('Error (\theta_1 - \theta_1_{actual}) [degrees]');
grid on;

subplot(2,1,2);
plot(t, error_2, 'b', 'LineWidth', 2);
title('Position Error for Joint 2');
xlabel('Time (seconds)');
ylabel('Error (\theta_2 - \theta_2_{actual}) [degrees]');
grid on;
