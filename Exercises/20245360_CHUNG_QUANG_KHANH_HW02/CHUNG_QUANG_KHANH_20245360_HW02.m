%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Student name: CHUNG QUANG KHANH
% Student ID:   20245360
% Homework 02
% Prof. KANG

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of D-H Parameters
% Link twist  % Link length   % Link offset   % Joint angle
alpha0 =   0;   a0 = 0;         d1 = 0;         theta1 = 30; % -> {1}
alpha1 = -90;   a1 = 0;         d2 = 0;         theta2 = 30; % -> {2}
alpha2 =   0;   a2 = 1;         d3 = 0.5;       theta3 = 45; % -> {3}
alpha3 = -90;   a3 = 0.3;       d4 = 1;         theta4 = 90; % -> {4}
alpha4 =  90;   a4 = 0;         d5 = 0;         theta5 = 30; % -> {5}
alpha5 = -90;   a5 = 0;         d6 = 0;         theta6 = 30; % -> {6}

alphaT =   0;   aT = 0;         dT = 0.3;       thetaT =  0; % -> {T}

% Compute transformation matrix T from D-H Parameters
T01 = DH_param(alpha0, a0, d1, theta1); % {0} -> {1}
T12 = DH_param(alpha1, a1, d2, theta2); % {1} -> {2}
T23 = DH_param(alpha2, a2, d3, theta3); % {2} -> {3}
T34 = DH_param(alpha3, a3, d4, theta4); % {3} -> {4}
T45 = DH_param(alpha4, a4, d5, theta5); % {4} -> {5}
T56 = DH_param(alpha5, a5, d6, theta6); % {5} -> {6}
T6T = DH_param(alphaT, aT, dT, thetaT); % {6} -> {Tool}
T0T = T01*T12*T23*T34*T45*T56*T6T;  % {0} -> {Tool}

disp("a) Transformation matrix from {0} to {Tool} is as follows:");
disp(T0T);

%   a) Transformation matrix from {0} to {Tool} is as follows:

%       -0.0669    0.9268   -0.3696   -1.1624
%       -0.9820   -0.1268   -0.1402    1.1366
%       -0.1768    0.3536    0.9186   -0.6437
%        0         0         0         1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T06 = T01*T12*T23*T34*T45*T56;  % {0} -> {6}
P06 = T06(1:3, 4);
% P06 = [ -1.0516;
%          1.1787;
%         -0.9192 ];

R06 = T06(1:3, 1:3);
% R06 = [ -0.0669    0.9268   -0.3696;
%         -0.9820   -0.1268   -0.1402;
%         -0.1768    0.3536    0.9186 ];

% DH parameter
alpha = [alpha0 alpha1 alpha2 alpha3 alpha4 alpha5];
a = [a0 a1 a2 a3 a4 a5];
d = [d1 d2 d3 d4 d5 d6];

% Find joint angles for frames 1, 2, and 3
P = P06 - d(6)*R06(1:3, 3);
C1 = sqrt(P(1)^2+P(2)^2);
C2 = P(3)-d(1);
C3 = sqrt(C1^2+C2^2);
C4 = sqrt(a(3)^2+d(4)^2);
D1 = d(2)/C1;
D2 = (C3^2+a(2)^2-C4^2)/(2*a(2)*C3);
D3 = (a(2)^2+C4^2-C3^2)/(2*a(2)*C4);

aa1 = atan2d(D1,sqrt(abs(1-D1^2)));
aa2 = atan2d(sqrt(abs(1-D2^2)),D2);
b = atan2d(sqrt(abs(1-D3^2)),D3);
p1 = atan2d(P(2),P(1));
p2 = atan2d(C2,C1);

ik_theta1 = p1-aa1;
ik_theta2 = round(aa2-p2);
ik_theta3 = round(b-90);
ik_theta = [ik_theta1 ik_theta2 ik_theta3];

% Apply forward kinematics at first three joints
T = [];
for n = 1:3
    Txy = DH_param(alpha(n), a(n), d(n), ik_theta(n));    
    T = [T; {Txy}];
end

T03 = T{1}*T{2}*T{3};
T36 = inv(T03)*T06;

% Find joint angles for frame 4, 5, and 6
ik_theta4 = round(atan2d(T36(2,3),T36(1,3)));
ik_theta5 = round(atan2d(sqrt(abs(1-T36(3,3)^2)),T36(3,3)));
ik_theta6 = atan2d(T36(3,2),-T36(3,1));
ik_theta_all = [ik_theta1 ik_theta2 ik_theta3 ik_theta4 ik_theta5 ik_theta6];

disp("b) Inverse Kinematic with the previous Transformation matrix is as follows:");
disp("  Theta1     Theta2    Theta3   Theta4     Theta5   Theta6");
disp(ik_theta_all);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%









                   