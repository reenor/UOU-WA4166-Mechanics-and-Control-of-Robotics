%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Student name: CHUNG QUANG KHANH
% Student ID:   20245360
% Homework:     07
% Professor:    KANG

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc, clearvars, close all

% Define Parameters
m1 = 5; m2 = 5;  % masses in kg
l1 = 0.5; l2 = 0.5;  % link lengths in meters
theta1_0 = 30; theta2_0 = 120; % initial joint angles (degrees)
theta1_tf = 150; theta2_tf = 30; % final joint angles (degrees)
theta1_dot_0 = 0; theta2_dot_0 = 0; % initial joint velocities (degrees/s)
g = 9.81; % Gravitational acceleration (m/s^2)

% CTC Controller Gains (Tune these values)
Kp1 = 15; Kd1 = 7;
Kp2 = 1; Kd2 = 0.9;

% Time Parameters
t0 = 0; tf = 2; % start and end time
dt = 0.01; % time step
time = t0:dt:tf;

% Define Trajectories for Joint 1 and Joint 2 (in degrees)
theta1 = @(t) (t < 0.5) .* (80*t.^2 + 30) + ...        % Parabolic path 1 (0.5s)
              (t >= 0.5 & t < 1.5) .* (80*t + 10) + ... % Linear path (1s)
              (t >= 1.5) .* (-80*t.^2 + 320*t -170);   % Parabolic path 2 (0.5s)

theta2 = @(t) (t < 0.5) .* (-60*t.^2 + 120) + ...        % Parabolic path 1 (0.5s)
              (t >= 0.5 & t < 1.5) .* (-60*t + 135) + ... % Linear path (1s)
              (t >= 1.5) .* (240*t.^3 - 1260*t.^2 + 2160*t - 1170); % Parabolic path 2 (0.5s)

% Initialize States (in degrees)
theta1_actual = theta1_0; theta2_actual = theta2_0;
theta1_dot_actual = theta1_dot_0; theta2_dot_actual = theta2_dot_0;

% Initialize Arrays for Results (in degrees)
theta1_results = zeros(length(time), 1);
theta2_results = zeros(length(time), 1);
theta1_desired = zeros(length(time), 1);
theta2_desired = zeros(length(time), 1);
theta1_error = zeros(length(time), 1);
theta2_error = zeros(length(time), 1);

% Helper function: Mass (inertia) matrix for 2-link arm
M = @(theta1, theta2) [m1*l1^2 + m2*(l1^2 + l2^2 + 2*l1*l2*cos(deg2rad(theta2))), m2*(l2^2 + l1*l2*cos(deg2rad(theta2)));
                      m2*(l2^2 + l1*l2*cos(deg2rad(theta2))), m2*l2^2];

% Helper function: Coriolis and centrifugal forces matrix
C = @(theta1, theta2, theta1_dot, theta2_dot) [-m2*l1*l2*sin(deg2rad(theta2))*theta2_dot, -m2*l1*l2*sin(deg2rad(theta2))*(theta1_dot + theta2_dot);
                                                 m2*l1*l2*sin(deg2rad(theta2))*theta1_dot, 0];

% Helper function: Gravity vector
G = @(theta1, theta2) [m1*g*(l1/2)*sin(deg2rad(theta1)) + m2*g*(l1*sin(deg2rad(theta1)) + (l2/2)*sin(deg2rad(theta1 + theta2)));
                       m2*g*(l2/2)*sin(deg2rad(theta1 + theta2))];

% CTC Control Simulation Loop
for i = 1:length(time)
    % Get the desired trajectories and velocities (in degrees)
    theta1_des = theta1(time(i)); 
    theta2_des = theta2(time(i));
    
    % Store desired angles
    theta1_desired(i) = theta1_des;
    theta2_desired(i) = theta2_des;
    
    % Compute desired velocities (first derivatives)
    if time(i) < 0.5
        theta1_dot_des = 160*time(i); % derivative of 80t^2 + 30
    elseif time(i) >= 0.5 && time(i) < 1.5
        theta1_dot_des = 80; % constant velocity for linear part
    else
        theta1_dot_des = -160*time(i) + 320; % derivative of -80t^2 + 320t -170
    end
    
    if time(i) < 0.5
        theta2_dot_des = -120*time(i); % derivative of -60t^2 + 120
    elseif time(i) >= 0.5 && time(i) < 1.5
        theta2_dot_des = -60; % constant velocity for linear part
    else
        theta2_dot_des = 720*time(i)^2 - 2520*time(i) + 2160; % derivative of cubic
    end
    
    % Compute desired accelerations (second derivatives)
    theta1_ddot_des = 160; % constant acceleration (for simplicity)
    theta2_ddot_des = -120; % constant acceleration (for simplicity)
    
    % Compute the current mass (inertia) matrix, Coriolis and gravity
    M_matrix = M(theta1_actual, theta2_actual);
    C_matrix = C(theta1_actual, theta2_actual, theta1_dot_actual, theta2_dot_actual);
    G_vector = G(theta1_actual, theta2_actual);
    
    % Compute the control torques (CTC Law)
    tau_control = M_matrix * ( [theta1_ddot_des; theta2_ddot_des] + ...
        [Kp1 * (theta1_des - theta1_actual) + Kd1 * (theta1_dot_des - theta1_dot_actual); 
         Kp2 * (theta2_des - theta2_actual) + Kd2 * (theta2_dot_des - theta2_dot_actual)] ) + ...
        C_matrix * [theta1_dot_actual; theta2_dot_actual] + G_vector;
    
    % Simulate the Dynamics (simplified by using torques and updating states)
    % Here we update velocities and positions using the torques computed by CTC.
    theta1_dot_actual = theta1_dot_actual + tau_control(1) * dt;
    theta2_dot_actual = theta2_dot_actual + tau_control(2) * dt;
    
    theta1_actual = theta1_actual + theta1_dot_actual * dt;
    theta2_actual = theta2_actual + theta2_dot_actual * dt;

    % Store simulated results
    theta1_results(i) = theta1_actual;
    theta2_results(i) = theta2_actual;
    
    % Calculate errors
    theta1_error(i) = theta1_des - theta1_actual;
    theta2_error(i) = theta2_des - theta2_actual;
end

% Plot desired and simulated angles for Joint 1 and Joint 2
figure;

% Plot Joint 1
subplot(3,1,1);
plot(time, theta1_desired, 'r--', 'LineWidth', 1.5); % Desired trajectory (red dashed line)
hold on;
plot(time, theta1_results, 'b-', 'LineWidth', 1.5);   % Simulated trajectory (blue solid line)
title('Joint 1 Angle');
xlabel('Time (s)');
ylabel('Theta1 (degrees)');
legend('Desired', 'Simulated');

% Plot Joint 2
subplot(3,1,2);
plot(time, theta2_desired, 'r--', 'LineWidth', 1.5); % Desired trajectory (red dashed line)
hold on;
plot(time, theta2_results, 'b-', 'LineWidth', 1.5);   % Simulated trajectory (blue solid line)
title('Joint 2 Angle');
xlabel('Time (s)');
ylabel('Theta2 (degrees)');
legend('Desired', 'Simulated');

% Plot Errors for Joint 1 and Joint 2 together
subplot(3,1,3);
plot(time, theta1_error, 'r-', 'LineWidth', 1.5); % Error for Joint 1 (red line)
hold on;
plot(time, theta2_error, 'b-', 'LineWidth', 1.5); % Error for Joint 2 (blue line)
title('Position Error for Joint 1 and Joint 2');
xlabel('Time (s)');
ylabel('Error (degrees)');
legend('Joint 1 Error', 'Joint 2 Error');
grid on;
