%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Student name: CHUNG QUANG KHANH
% Student ID:   20245360
% Homework 03
% Prof. KANG

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc, clearvars, close all

dt = 1;
num_sec = 5;
t = 0:dt:num_sec;

% Gyroscope readings
w_x = sin(2*t)/100;
w_y = cos(5*t)/100;
w_z = cos(t)/10;

% Euler angle: psi -> theta -> phi
psi = zeros(1,11);   
theta = zeros(1,11);
phi = zeros(1,11);

% Initial posture at (10, 10, 10) degree
psi(1) = 10;
theta(1) = 10;
phi(1) = 10;

% Time rate of change of psi, theta, phi
psi_dot = zeros(1,11);
theta_dot = zeros(1,11);
phi_dot = zeros(1,11);
 
for i = 1:num_sec
    B = [ -sind(theta(i))                   0               1;
           sind(phi(i))*cosd(theta(i))      cosd(phi(i))    0;
           cosd(phi(i))*cosd(theta(i))     -sind(phi(i))    0    ];
    
    W = [w_x(i); w_y(i); w_z(i)];
    
    % Compute time rate of change
    R = inv(B)*W;
    
    psi_dot(i) = R(1);
    theta_dot(i) = R(2);
    phi_dot(i) = R(3);
    
    % Intergration for next posture
    psi(i+1) = psi(i) + dt*psi_dot(i);
    theta(i+1) = theta(i) + dt*theta_dot(i);
    phi(i+1) = phi(i) + dt*phi_dot(i);
end
 
% Define the cubic, 4 face around
X = [0 0 0 0 0 10; 10 0 10 10 10 10; 10 0 10 10 10 10; 0 0 0 0 0 10];
Y = [0 0 0 0 10 0; 0 10 0 0 10 10; 0 10 10 10 10 10; 0 0 10 10 10 0];
Z = [0 0 10 0 0 0; 0 0 10 0 0 0; 10 10 10 0 10 10; 10 10 10 0 10 10];
 
xc = 0; yc = 0; zc = 0;     % coordinated of the center
L = 1;                      % cube size (length of an edge)
alpha = 1;                  % transparency (max=1=opaque)
 
% Draw the cubic at the initial
C = [0.1 0.4 0.8 0.8 0.4 0.1];  % color/face
s = 0;
 
X = L*(X-0.5) + xc;
Y = L*(Y-0.5) + yc;
Z = L*(Z-0.5) + zc;  
 
figure();
fill3(X,Y,Z,C); % draw cube
axis equal;
AZ=-20;         % azimuth
EL=25;          % elevation
view(AZ,EL);    % orientation of the axes
    
% Draw the cubic by time
for j = 1:num_sec
    a11 = cosd(psi(j))*cosd(theta(j))*cosd(phi(j)) - sind(psi(j))*sind(phi(j));
    a21 = sind(psi(j))*cosd(theta(j))*cosd(phi(j)) + cosd(psi(j))*sind(phi(j));
    a31 = -sind(theta(j))*cosd(phi(j));

    a12 = -cosd(psi(j))*cosd(theta(j))*sind(phi(j)) - sind(psi(j))*cosd(phi(j));
    a22 = -sind(psi(j))*cosd(theta(j))*sind(phi(j)) + cosd(psi(j))*cosd(phi(j));
    a32 = sind(theta(j))*sind(phi(j));

    a13 = cosd(psi(j))*sind(theta(j));
    a23 = sind(psi(j))*sind(theta(j));
    a33 = cosd(theta(j));

    K = [a11, a12, a13; a21, a22, a23; a31, a32, a33];

    % Row 1
    temp3 = [X(1,:); Y(1,:); Z(1,:)];
    temp4 = K*temp3;

    X(1,:) = temp4(1,:);
    Y(1,:) = temp4(2,:);
    Z(1,:) = temp4(3,:);

    % Row 2
    temp5 = [X(2,:); Y(2,:); Z(2,:)];
    temp6 = K*temp5;

    X(2,:) = temp6(1,:);
    Y(2,:) = temp6(2,:);
    Z(2,:) = temp6(3,:);

    % Row 3
    temp7 = [X(3,:); Y(3,:); Z(3,:)];
    temp8 = K*temp7;

    X(3,:) = temp8(1,:);
    Y(3,:) = temp8(2,:);
    Z(3,:) = temp8(3,:);
    % Row 4
    temp9 = [X(4,:); Y(4,:); Z(4,:)];
    temp10 = K*temp9;

    X(4,:) = temp10(1,:);
    Y(4,:) = temp10(2,:);
    Z(4,:) = temp10(3,:);

    s = 0;

    X = L*(X-s) + xc;
    Y = L*(Y-s) + yc;
    Z = L*(Z-s) + zc; 

    figure();

    fill3(X,Y,Z,C); % draw cube
    axis equal;
    AZ=-20;         % azimuth
    EL=25;          % elevation
    view(AZ,EL);    % orientation of the axes  
 end
 