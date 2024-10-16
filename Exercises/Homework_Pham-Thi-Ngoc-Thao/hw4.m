clear all;
clc;
 
%DH parameter
alpha =  [0 -pi/2 0 -pi/2 pi/2 -pi/2] ; %link twist
a = [0 0 1 0.3 0 0]; %link length
d = [0 0  0.5 1 0 0]; %link offset
theta = [pi/6 pi/6 pi/6 pi/6 pi/6 pi/6]; % joint variable
 
omega=[0;0;0.1];
 
P6_T = [0 0 0.3 1];
 
%Apply forward kinematics joints
T = [];
R = [];
for n = 1:6
       matT = [cos(theta(n)) -sin(theta(n)) 0 a(n);
           sind(theta(n))*cos(alpha(n))  cos(theta(n))*cos(alpha(n)) -sin(alpha(n)) -sin(alpha(n))*d(n);
           sin(theta(n))*sin(alpha(n))  cos(theta(n))*sin(alpha(n)) cos(alpha(n)) cos(alpha(n))*d(n);
           0 0 0 1];
        T = [T; {matT}];
        R = [R; {matT(1:3,1:3)}];
end
 
R = [R; {[1 0 0; 0 1 0; 0 0 1]}];
    
P_0_1 = [0; 0; 0];
P_1_2 = [0;0;0];
P_2_3 = [a(3);0;d(3)];
P_3_4=[a(4);d(4);0];
P_4_5=[ 0;0;0];
P_5_6=[0;0;0];
P_6_t=[0;0;0.3];
 
w_0_0=[0;0;0];
v_0_0=[0;0;0];
 
w_1_1=R{1}'*w_0_0+[0;0;0.1];
v_1_1=R{1}'*(v_0_0+cross(w_0_0,P_0_1));
 
w_2_2=R{2}'*w_1_1+[0;0;0.1];
v_2_2=R{2}'*(v_1_1+cross(w_1_1,P_1_2));
 
w_3_3=R{3}'*w_2_2+[0;0;0.1];
v_3_3=R{3}'*(v_2_2+cross(w_2_2,P_2_3));
 
w_4_4=R{4}'*w_3_3+[0;0;0.1];
v_4_4=R{4}'*(v_3_3+cross(w_3_3,P_3_4));
 
w_5_5=R{5}'*w_4_4+[0;0;0.1];
v_5_5=R{5}'*(v_4_4+cross(w_4_4,P_4_5));
 
w_6_6=R{6}'*w_5_5+[0;0;0.1];
v_6_6=R{6}'*(v_5_5+cross(w_5_5,P_5_6));
 
w_t_t=R{7}'*w_6_6+0;
v_t_t=R{7}'*(v_6_6+cross(w_6_6,P_6_t));
 
w_0_t=R{1}*R{2}*R{3}*R{4}*R{5}*R{6}*R{7}*w_6_6
v_0_t=R{1}*R{2}*R{3}*R{4}*R{5}*R{6}*R{7}*v_t_t

 
V_0=[v_0_t;w_0_t]

T0_6 = T{1}*T{2}*T{3}*T{4}*T{5}*T{6}
P0_T = T0_6.*P6_T

%%%%%%%%%%%%% Jacobian
P_0_t=P0_T(1:3, 4);

k_0_1=T{1}(1:3,3) ;
P0_1=T{1}(1:3,4) ;
J1=[cross(k_0_1,(P_0_t-P0_1));k_0_1]

T0_2=T{1}*T{2}
k_0_2=T0_2(1:3,3) ;
P0_2=T0_2(1:3,4) ;
J2=[cross(k_0_2,(P_0_t-P0_2));k_0_2]

T0_3=T{1}*T{2}*T{3}
k_0_3=T0_3(1:3,3) ;
P0_3=T0_3(1:3,4) ;
J3=[cross(k_0_3,(P_0_t-P0_3));k_0_3]

T0_4=T{1}*T{2}*T{3}*T{4}
k_0_4=T0_4(1:3,3) ;
P0_4=T0_4(1:3,4) ;
J4=[cross(k_0_4,(P_0_t-P0_4));k_0_4]
%
T0_5=T{1}*T{2}*T{3}*T{4}*T{5}
k_0_5=T0_5(1:3,3) ;
P0_5=T0_5(1:3,4) ;
J5=[cross(k_0_5,(P_0_t-P0_5));k_0_5]

T0_6=T{1}*T{2}*T{3}*T{4}*T{5}*T{6}
k_0_6=T0_6(1:3,3) ;
P0_6=T0_6(1:3,4) ;
J6=[cross(k_0_6,(P_0_t-P0_6));k_0_6]
J=[J1 J2 J3 J4 J5 J6]

%%%%%%%%%%%%%%%%%%%%%%%%%Inverse velocity%
th_dot=inv(J)*V_0

