clear all; clc

%Given condition
a = [0 0 1 0.3 0 0 0]; %a(7)은 Tip of the arm의 값
d = [0 0 0.5 1 0 0 0.3]; %d(7)은 Tip of the arm의 값

% 위의 결과값
T0_T = [0.2522    0.1044   -0.9620   -0.4087;
       -0.7874   -0.5558   -0.2667    0.4280;
       -0.5625    0.8248   -0.0580   -1.2772;
          0         0         0       1.0000];

% thetaT = 0 degree and 6PT = [0 0 0.3]T에 의해서 다음과 같이 구해짐
T6_T = [1           0         0       0;
        0           1         0       0;
        0           0         1       0.3;
        0           0         0       1];

T0_6 = T0_T*(inv(T6_T));

a0  = a(1); a1  = a(2); a2  = a(3);
a3  = a(4); a4  = a(5); a5  = a(6);

d1  = d(1); d2  = d(2); d3  = d(3);
d4  = d(4); d5  = d(5); d6  = d(6);

r11 = T0_6(1,1); r12 = T0_6(1,2); r13 = T0_6(1,3);
r21 = T0_6(2,1); r22 = T0_6(2,2); r23 = T0_6(2,3);
r31 = T0_6(3,1); r32 = T0_6(3,2); r33 = T0_6(3,3);

px  = T0_6(1,4); py  = T0_6(2,4); pz  = T0_6(3,4);

theta = zeros(8:6);
theta_deg = zeros(8:6);

%theta1
theta(1:4,1) = (atan2(py, px) - atan2(d3, sqrt(px^2+py^2-d3^2)));
theta(5:8,1) = (atan2(py, px) - atan2(d3, -sqrt(px^2+py^2-d3^2)));

theta_deg = theta*180/pi; %Convert rad to deg

%theta3
K = (px^2+py^2+pz^2-a2^2-a3^2-d3^2-d4^2)/(2*a2);

theta(1:2,3) = (atan2(a3, d4) - atan2(K, sqrt(a3^2+d4^2-K^2)));
theta(3:4,3) = (atan2(a3, d4) - atan2(K, -sqrt(a3^2+d4^2-K^2)));
theta(5:6,3) = (atan2(a3, d4) - atan2(K, sqrt(a3^2+d4^2-K^2)));
theta(7:8,3) = (atan2(a3, d4) - atan2(K, -sqrt(a3^2+d4^2-K^2)));

theta_deg = theta*180/pi; %Convert rad to deg

%theta2
s23 = zeros(1,8);
c23 = zeros(1,8);

for i = 1:8
    s23(i) = ((-a3-a2*cos(theta(i,3)))*pz+(cos(theta(i,1))*px+sin(theta(i,1))*py)*(a2*sin(theta(i,3))-d4))/...
        (pz^2+(cos(theta(i,1))*px+sin(theta(i,1))*py)^2);
    c23(i) = ((a3+a2*cos(theta(i,3)))*(cos(theta(i,1))*px+sin(theta(i,1))*py)-pz*(d4-a2*sin(theta(i,3))))/...
        (pz^2+(cos(theta(i,1))*px+sin(theta(i,1))*py)^2); 
end


theta23 = zeros(1,8);
for i = 1:8
    theta23(i) = atan2(s23(i), c23(i));
    theta(i,2) = theta23(i) - theta(i,3);
end

theta_deg = theta*180/pi; %Convert rad to deg
 
%theta4
for i=1:4
    theta(2*i-1,4) = atan2((-r13)*sin(theta(2*i-1,1))+r23*cos(theta(2*i-1,1)),...
        -r13*cos(theta(2*i-1,1))*c23(2*i-1)-r23*sin(theta(2*i-1,1))*c23(2*i-1)+r33*s23(2*i-1));
    theta(2*i,4) = theta(2*i-1,4)+pi;
end


theta_deg = theta*180/pi; %Convert rad to deg

%theta5
s5 = zeros(1,8);
c5 = zeros(1,8);
for i=1:4
    s5(2*i-1) = -(r13*(cos(theta(2*i-1,1))*c23(2*i-1)*cos(theta(2*i-1,4))+...
        sin(theta(2*i-1,1))*sin(theta(2*i-1,4)))+r23*(sin(theta(2*i-1,1))*c23(2*i-1)*cos(theta(2*i-1,4))-...
        cos(theta(2*i-1,1))*sin(theta(2*i-1,4)))-r33*(s23(2*i-1)*cos(theta(2*i-1,4))));
    c5(2*i-1) = r13*((-1)*cos(theta(2*i-1,1))*s23(2*i-1))+r23*((-1)*sin(theta(2*i-1,1))*s23(2*i-1))+...
        r33*((-1)*c23(2*i-1));

    theta(2*i-1,5) = atan2(s5(2*i-1), c5(2*i-1));
    theta(2*i,5) = (-1)*theta(2*i-1,5);
end

theta_deg = theta*180/pi; %Convert rad to deg

%theta6
s6 = zeros(1,8);
c6 = zeros(1,8);
for i=1:4
    s6 = (-1)*r11*(cos(theta(2*i-1,1))*c23(2*i-1)*sin(theta(2*i-1,4))-sin(theta(2*i-1,1))*cos(theta(2*i-1,4)))-...
        r21*(sin(theta(2*i-1,1))*c23(2*i-1)*sin(theta(2*i-1,4))+cos(theta(2*i-1,1))*cos(theta(2*i-1,4)))+...
        r31*(s23(2*i-1)*sin(theta(2*i-1,4)));
    c6 = r11*((cos(theta(2*i-1,1))*c23(2*i-1)*cos(theta(2*i-1,4))+sin(theta(2*i-1,1))*sin(theta(2*i-1,4)))*cos(theta(2*i-1,5))-...
        cos(theta(2*i-1,1))*s23(2*i-1)*sin(theta(2*i-1,5)))+r21*((sin(theta(2*i-1,1))*c23(2*i-1)*cos(theta(2*i-1,4))-...
        cos(theta(2*i-1,1))*sin(theta(2*i-1,4)))*cos(theta(2*i-1,5))-sin(theta(2*i-1,1))*s23(2*i-1)*sin(theta(2*i-1,5)))-...
        r31*(s23(2*i-1)*cos(theta(2*i-1,4))*cos(theta(2*i-1,5))+c23(2*i-1)*sin(theta(2*i-1,5)));

    theta(2*i-1,6) = atan2(s5(2*i-1), c5(2*i-1));
    theta(2*i,6) = theta(2*i-1,6) + pi;
end

theta_deg = theta*180/pi %Convert rad to deg



