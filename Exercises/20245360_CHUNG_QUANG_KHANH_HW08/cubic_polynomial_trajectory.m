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

% Polynomial coefficients for joint 1 (theta_1(t))
% Using the cubic polynomial equation: theta_1(t) = a_1*t^3 + b_1*t^2 + c_1*t + d_1

% Boundary conditions:
% theta_1(0) = theta_1_0, theta_dot_1(0) = theta_dot_1_0
% theta_1(t_f) = theta_1_f, theta_dot_1(t_f) = theta_dot_1_f

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

% Polynomial coefficients for joint 2 (theta_2(t))
% Using the cubic polynomial equation: theta_2(t) = a_2*t^3 + b_2*t^2 + c_2*t + d_2

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

% Calculate joint angles theta_1(t) and theta_2(t)
theta_1_t = coeffs_1(1)*t.^3 + coeffs_1(2)*t.^2 + coeffs_1(3)*t + coeffs_1(4);
theta_2_t = coeffs_2(1)*t.^3 + coeffs_2(2)*t.^2 + coeffs_2(3)*t + coeffs_2(4);

% Plot the trajectories of both joints
figure;
subplot(2,1,1);
plot(t, theta_1_t, 'r', 'LineWidth', 2);
title('Joint 1 Trajectory (\theta_1(t))');
xlabel('Time (seconds)');
ylabel('Theta_1 (degrees)');
grid on;

subplot(2,1,2);
plot(t, theta_2_t, 'b', 'LineWidth', 2);
title('Joint 2 Trajectory (\theta_2(t))');
xlabel('Time (seconds)');
ylabel('Theta_2 (degrees)');
grid on;

% Display the coefficients for both joints
disp('Coefficients for Joint 1 (theta_1):');
disp(coeffs_1);

disp('Coefficients for Joint 2 (theta_2):');
disp(coeffs_2);
