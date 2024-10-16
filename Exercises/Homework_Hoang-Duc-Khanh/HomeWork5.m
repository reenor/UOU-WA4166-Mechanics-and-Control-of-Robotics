clear all
clc
%% Transformation%%%%%%%%
 
t = 0:1:5;   %5 second
delta = 1;
a = zeros(1,11);   %Alpha
b = zeros(1,11);   %Beta`
g = zeros(1,11);   %Gamma
 
a_dot = zeros(1,11);   %Velocity of Alpha 
b_dot = zeros(1,11);   %Velocity of Beta
g_dot = zeros(1,11);   %Velocity of Gamma
 
w_x = sin(t)/20;    %Gyroscope at x coordinate
w_y = sin(2*t)/20;  %Gyroscope at y coordinate
w_z = cos(t)/10;    %Gyroscope at z coordinate
 
%Compute Alpha, Beta, Gamma from data of Gyroscopes
a(1) = pi/18;
b(1) = pi/18;
g(1) = pi/18;
 
for i = 1:5
    %Compute Jacobian matrix
   E = [0, -sin(a(i)), cos(a(i))*sin(b(i));
          0, cos(a(i)), sin(a(i))*sin(b(i));
          1, 0, cos(b(i))];
       
    W = [w_x(i); w_y(i); w_z(i)];
    
    %Compute a_dot, b_dot, g-dot
    V = inv(E)*W;
    
    a_dot(i) = V(1);
    b_dot(i) = V(2);
    g_dot(i) = V(3);
    
    %Intergration for alpha, beta, gamma next cycle
    a(i+1) = a(i) + delta*a_dot(i);
    b(i+1) = b(i) + delta*b_dot(i);
    g(i+1) = g(i) + delta*g_dot(i);
end
 
%Define the cubic, 4 face around
X = [0 0 0 0 0 10; 10 0 10 10 10 10; 10 0 10 10 10 10; 0 0 0 0 0 10];
Y = [0 0 0 0 10 0; 0 10 0 0 10 10; 0 10 10 10 10 10; 0 0 10 10 10 0];
Z = [0 0 10 0 0 0; 0 0 10 0 0 0; 10 10 10 0 10 10; 10 10 10 0 10 10];
 
xc=0; yc=0; zc=0;    % coordinated of the center
L=1;                 % cube size (length of an edge)
alpha=0.8;           % transparency (max=1=opaque)
 
%Draw the cubic at the initial
C= [0.1 0.5 0.8 0.8 0.1 0.9];  % color/face
 s = 0;
 
   X = L*(X-0.5) + xc;
   Y = L*(Y-0.5) + yc;
   Z = L*(Z-0.5) + zc;  
 
    figure();
    fill3(X,Y,Z,C,'FaceAlpha',alpha);    % draw cube
    axis equal;
    AZ=-20;         % azimuth
    EL=25;          % elevation
    view(AZ,EL);    % orientation of the axes  
    
%Draw the cubic by time
for j=1:5
    a11 = cos(a(j))*cos(b(j))*cos(g(j)) - sin(a(j))*sin(g(j));
    a21 = sin(a(j))*cos(b(j))*cos(g(j)) + cos(a(j))*sin(g(j));
    a31 = -sin(b(j))*cos(g(j));
    
    a12 = -cos(a(j))*cos(b(j))*sin(g(j)) - sin(a(j))*cos(g(j));
    a22 = -sin(a(j))*cos(b(j))*sin(g(j)) + cos(a(j))*cos(g(j));
    a32 = sin(b(j))*sin(g(j));
    
    a13 = cos(a(j))*sin(b(j));
    a23 = sin(a(j))*sin(b(j));
    a33 = cos(b(j));
    
    K = [a11, a12, a13; a21, a22, a23; a31, a32, a33];
    
    %Row 1
    temp3 = [X(1,:); Y(1,:); Z(1,:)];
    temp4 = K*temp3;
    
    X(1,:) = temp4(1,:);
    Y(1,:) = temp4(2,:);
    Z(1,:) = temp4(3,:);
    
    %Row 2
    temp5 = [X(2,:); Y(2,:); Z(2,:)];
    temp6 = K*temp5;
    
    X(2,:) = temp6(1,:);
    Y(2,:) = temp6(2,:);
    Z(2,:) = temp6(3,:);
    
    %Row 3
    temp7 = [X(3,:); Y(3,:); Z(3,:)];
    temp8 = K*temp7;
    
    X(3,:) = temp8(1,:);
    Y(3,:) = temp8(2,:);
    Z(3,:) = temp8(3,:);
    %Row 4
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
    
    fill3(X,Y,Z,C,'FaceAlpha',alpha);    % draw cube
    axis equal;
    AZ=-20;         % azimuth
    EL=25;          % elevation
    view(AZ,EL);    % orientation of the axes  
 end

