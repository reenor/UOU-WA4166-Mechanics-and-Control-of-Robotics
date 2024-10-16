clear all;
clc;

%DH parameter
alpha =  [0 -90 0 -90 90 -90] ; %link twist
a = [0 0 1 0.3 0 0]; %link length
d = [0 0  0.5 1 0 0]; %link offset
theta = [45 60 45 60 45 30]; % joint variable

P6_T = [0 0 0.2 1];

%Apply forward kinematics joints
T = [];
for n = 1:6
       matT = [cosd(theta(n)) -sind(theta(n)) 0 a(n);
           sind(theta(n))*cosd(alpha(n))  cosd(theta(n))*cosd(alpha(n)) -sind(alpha(n)) -sind(alpha(n))*d(n);
           sind(theta(n))*sind(alpha(n))  cosd(theta(n))*sind(alpha(n)) cosd(alpha(n)) cosd(alpha(n))*d(n);
           0 0 0 1];
        T = [T; {matT}];
end


T0_6 = T{1}*T{2}*T{3}*T{4}*T{5}*T{6}
P0_T = T0_6*P6_T

