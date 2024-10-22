%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Student name: CHUNG QUANG KHANH
% Student ID:   20245360
% Homework:     04
% Professor:    KANG

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc, clearvars, close all


%%%%%%%%%%%%%%%%% 1. Find the linear and angular velocities of the tool through velocity propagation
% Definition of D-H Parameters
% Link twist        % Link length   % Link offset   % Joint angle
alpha0 =   0;       a0 = 0;         d1 = 0;         theta1 = 30; % -> {1}
alpha1 = -90;       a1 = 0;         d2 = 0;         theta2 = 30; % -> {2}
alpha2 =   0;       a2 = 1;         d3 = 0.5;       theta3 = 45; % -> {3}
alpha3 = -90;       a3 = 0.3;       d4 = 1;         theta4 = 90; % -> {4}
alpha4 =  90;       a4 = 0;         d5 = 0;         theta5 = 30; % -> {5}
alpha5 = -90;       a5 = 0;         d6 = 0;         theta6 = 30; % -> {6}
alphaT =   0;       aT = 0;         dT = 0.3;       thetaT =  0; % -> {T}

% Construct Rotation matrix and Position vector from D-H Parameters
%       Rxy: Rotation matrix of frame x with respect to frame y
%       Pxy: Position of frame x with respect to frame y
[R10, P10] = DH_2_R_P(alpha0, a0, d1, theta1);
[R21, P21] = DH_2_R_P(alpha1, a1, d2, theta2);
[R32, P32] = DH_2_R_P(alpha2, a2, d3, theta3);
[R43, P43] = DH_2_R_P(alpha3, a3, d4, theta4);
[R54, P54] = DH_2_R_P(alpha4, a4, d5, theta5);
[R65, P65] = DH_2_R_P(alpha5, a5, d6, theta6);
[RT6, PT6] = DH_2_R_P(alphaT, aT, dT, thetaT); % T is the tool frame

R = [R10, R21, R32, R43, R54, R65, RT6];
P = [P10, P21, P32, P43, P54, P65, PT6];
num = length(P);

omega_00 = [ 0 0 0 ]'; % Base frame {0} has zero angular velocity
vel_00   = [ 0 0 0 ]'; % Base frame {0} also has zero linear velocity
theta_dot = 0.1; % radians/second, given that all joint velocities are the same
Zh = [ 0 0 1 ]'; % Unit vector along Z axis

% Start computing from base frame {0} -> tool frame {T}
omega = omega_00; vel = vel_00;
for k = 1:num

    % Compute angular velocity
    omega = (R(:, k*3-2:k*3)' * omega) + (theta_dot * Zh); 

    % By convention, positive angular velocity indicates counter-clockwise rotation, while negative
    % is clockwise.
    disp(omega);

    % Compute linear velocity
    S = [  0            -omega(3)     omega(2);
           omega(3)      0           -omega(1);
          -omega(2)      omega(1)     0           ]; % Form skew-symmetric matrix from angular vel
    vel = R(:, k*3-2:k*3)' * (vel + S * P(:, k));
    disp(vel);
end

%%%%%%%%%%%%%%%%% 2. Find the Jacobian at that instant


