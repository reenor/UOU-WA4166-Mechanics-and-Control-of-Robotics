%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Student name: CHUNG QUANG KHANH
% Student ID:   20245360
% Homework:     04
% Professor:    KANG

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc, clearvars, close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% 1. Find the linear and angular velocities of the tool through velocity propagation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%       Rxy: Rotation matrix of frame y with respect to frame x
%       Pxy: Position of frame y with respect to frame x
[R01, P01] = DH_2_R_P(alpha0, a0, d1, theta1);
[R12, P12] = DH_2_R_P(alpha1, a1, d2, theta2);
[R23, P23] = DH_2_R_P(alpha2, a2, d3, theta3);
[R34, P34] = DH_2_R_P(alpha3, a3, d4, theta4);
[R45, P45] = DH_2_R_P(alpha4, a4, d5, theta5);
[R56, P56] = DH_2_R_P(alpha5, a5, d6, theta6);

[R6T, P6T] = DH_2_R_P(alphaT, aT, dT, thetaT); % T is the tool frame

R = [R01, R12, R23, R34, R45, R56, R6T];
P = [P01, P12, P23, P34, P45, P56, P6T];
num = length(P);

omega_00 = [ 0 0 0 ]'; % Base frame {0} has zero angular velocity
vel_00   = [ 0 0 0 ]'; % Base frame {0} also has zero linear velocity
theta_dot = 0.1; % radians/second, given that all joint velocities are the same
Zh = [ 0 0 1 ]'; % Unit vector along Z axis

% Start computing from base frame {0} -> tool frame {T}
omega = omega_00; vel = vel_00;
for k = 1:num

    prev_omega = omega;

    % Compute angular velocity
    if k ~= 7
        omega = (R(:, k*3-2:k*3)' * omega) + (theta_dot * Zh);
    else % k = 7, the tool doesn't have velocity
        omega = R(:, k*3-2:k*3)' * omega;
    end

    % By convention, positive angular velocity indicates counter-clockwise rotation, while negative
    % is clockwise.

    % Compute linear velocity
    S = [  0                -prev_omega(3)     prev_omega(2);
           prev_omega(3)     0                -prev_omega(1);
          -prev_omega(2)     prev_omega(1)     0           ]; % Form skew-symmetric matrix from angular vel
    vel = R(:, k*3-2:k*3)' * (vel + S * P(:, k));
end

disp('Linear and Angular velocities of the tool w.r.t frame {T}: ');
v_TT = [vel; omega]

disp('Linear and Angular velocities of the tool w.r.t frame {0}: ');
R0T = R01 * R12 * R23 * R34 * R45 * R56 * R6T;
omega_0T = R0T * omega;
vel_0T   = R0T * vel;
v_0T = [vel_0T; omega_0T]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% 2. Find the Jacobian at that instant
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute transformation matrix T from D-H Parameters
T01 = DH_2_T(alpha0, a0, d1, theta1); % {0} -> {1}
T12 = DH_2_T(alpha1, a1, d2, theta2); % {1} -> {2}
T23 = DH_2_T(alpha2, a2, d3, theta3); % {2} -> {3}
T34 = DH_2_T(alpha3, a3, d4, theta4); % {3} -> {4}
T45 = DH_2_T(alpha4, a4, d5, theta5); % {4} -> {5}
T56 = DH_2_T(alpha5, a5, d6, theta6); % {5} -> {6}
T6T = DH_2_T(alphaT, aT, dT, thetaT); % {6} -> {Tool}

T0T = T01*T12*T23*T34*T45*T56*T6T;  % {0} -> {Tool}
P0T = T0T(1:3, 4);

% Compute the 1st column of Jacobian matrix
k01 = T01(1:3, 3);
P01 = T01(1:3, 4);
J1  = [ cross( k01, (P0T - P01) ); k01 ];

% Compute the 2nd column of Jacobian matrix
T02 = T01 * T12;
k02 = T02(1:3, 3);
P02 = T02(1:3, 4);
J2 = [ cross( k02, (P0T - P02) ); k02];

% Compute the 3rd column of Jacobian matrix
T03 = T01 * T12 * T23;
k03 = T03(1:3, 3);
P03 = T03(1:3, 4);
J3 = [ cross( k03, (P0T - P03) ); k03];

% Compute the 4th column of Jacobian matrix
T04 = T01 * T12 * T23 * T34;
k04 = T04(1:3, 3);
P04 = T04(1:3, 4);
J4 = [ cross( k04, (P0T - P04) ); k04];

% Compute the 5th column of Jacobian matrix
T05 = T01 * T12 * T23 * T34 * T45;
k05 = T05(1:3, 3);
P05 = T05(1:3, 4);
J5 = [ cross( k05, (P0T - P05) ); k05];

% Compute the 6th column of Jacobian matrix
T06 = T01 * T12 * T23 * T34 * T45 * T56;
k06 = T06(1:3, 3);
P06 = T06(1:3, 4);
J6 = [ cross( k06, (P0T - P06) ); k06];

disp('Jacobian matrix:');
J = [J1 J2 J3 J4 J5 J6]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% 3. With the inverse of Jacobian and the obtained results, do velocity inverse
%%%%%%%     kinematics to find the joint velocities.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Velocity inverse kinematics to find the joint velocities:');
THETA_dot = inv(J) * v_0T
