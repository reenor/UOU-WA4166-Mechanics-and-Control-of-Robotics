clear all;
clc;

%DH parameter
alpha =  [0 -90 0 -90 90 -90] ; %link twist
a = [0 0 1 0.3 0 0]; %link length
d = [0 0  0.5 1 0 0]; %link offset
theta = [30 30 30 30 30 30]; % joint variable

P6_T = [0 0 0.3 1];

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
P0_T = T0_6.*P6_T

%---------------------------------------------------------------------------------

%calulate T0_T, T06 with the code in a.

%problem b: 8 set parameter
px = T0_6(1,4);
py = T0_6(2,4);
pz = T0_6(3,4);
r = T0_6(1:3,1:3);
th = zeros(6,8);
 
th(1,1:4) = atan2d(py,px) - atan2d(d(3), sqrt(px^2 + py^2 -d(3)^2));
th(1,5:8) = atan2d(py,px) - atan2d(d(3), -sqrt(px^2 + py^2 -d(3)^2));
 
K = (px^2 + py^2 + pz^2 - a(3)^2 - a(4)^2 - d(3)^2 - d(4)^2)/(2*a(3));
 
th(3,[1,2,5,6]) = atan2d(a(4),d(4)) - atan2d(K, sqrt(a(4)^2 + d(4)^2 - K^2));
th(3,[3,4,7,8]) = atan2d(a(4),d(4)) - atan2d(K, -sqrt(a(4)^2 + d(4)^2 - K^2));
 
th23 = atan2d((-a(4)-a(3)*cosd(th(3,:)))*pz + (cosd(th(1,:))*px + sind(th(1,:))*py).*(d(4)-a(3)*sind(th(3,:))),(a(3)*sind(th(3,:))-d(4))*pz+(a(4)+a(3)*cosd(th(3,:))).*(cosd(th(1,:))*px + sind(th(1,:))*py));
 
th(2,:) = th23 - th(3,:);
 
th(4,:) = atan2d(-r(1,3)*sind(th(1,:))+r(2,3)*cosd(th(1,:)),-r(1,3)*cosd(th(1,:)).*cosd(th23)-r(2,3)*sind(th(1,:)).*cosd(th23) + r(3,3)*sind(th23));
 
s5 = -r(1,3)*(cosd(th(1,:)).*cosd(th23).*cosd(th(4,:)) + sind(th(1,:)).*sind(th(4,:))) - r(2,3)*(sind(th(1,:)).*cosd(th23).*cosd(th(4,:)) - cosd(th(1,:)).*sind(th(4,:))) + r(3,3)*sind(th23).*cosd(th(4,:));
 
c5 = -r(1,3)*cosd(th(1,:)).*sind(th23) - r(2,3)*sind(th(1,:)).*sind(th23) - r(3,3)*cosd(th23);
 
th(5,:) = atan2d(s5,c5);
 
s6 = -r(1,1)*(cosd(th(1,:)).*cosd(th23).*sind(th(4,:)) - sind(th(1,:)).*cosd(th(4,:))) - r(2,1)*(sind(th(1,:)).*cosd(th23).*sind(th(4,:)) + cosd(th(1,:)).*cosd(th(4,:))) + r(3,1)*sind(th23).*sind(th(4,:));
 
c6 = r(1,1)*((cosd(th(1,:)).*cosd(th23).*cosd(th(4,:)) + sind(th(1,:)).*sind(th(4,:))).*cosd(th(5,:)) - cosd(th(1,:)).*sind(th23).*sind(th(5,:))) + r(2,1)*((sind(th(1,:)).*cosd(th23).*cosd(th(4,:)) - cosd(th(1,:)).*sind(th(4,:))).*cosd(th(5,:)) - sind(th(1,:)).*sind(th23).*sind(th(5,:))) - r(3,1)*(sind(th23).*cosd(th(4,:)).*cosd(th(5,:)) + cosd(th23).*sind(th(5,:)));
 
th(6,:) = atan2d(s6,c6);
 
th(4,[2 4 6 8]) = th(4,[2 4 6 8]) + 180;
th(5,[2 4 6 8]) = -th(5,[2 4 6 8]);
th(6,[2 4 6 8]) = th(6,[2 4 6 8]) + 180;
th