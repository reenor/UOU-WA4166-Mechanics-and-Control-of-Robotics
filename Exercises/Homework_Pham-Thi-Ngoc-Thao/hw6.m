clear all;
clc;

th1 = 30*pi/180;
th_dot1 = 0;
th2 = 150*pi/180;
th_dot2 = 0;

tht = [th1; th2];
tht_dot = [th_dot1; th_dot2];

thef1 = 150*pi/180;
thef_dot1 = 0;
thef2 = 30*pi/180;
thef_dot2 = 0;

thtf = [thef1; thef2];
thtf_dot = [thef_dot1; thef_dot2];

l1 = 0.5;
l2 = 0.5;
m1 = 10;
m2 = 5;

tf = 1;
t = 0;

a0 = tht;
a1 = tht_dot;
a2 = 3/tf^2*(thtf-tht) - 2/tf*tht_dot - 1/tf*thtf_dot;
a3 = -2/tf^3*(thtf-tht) + 1/tf^2*(thtf_dot+tht_dot);

% Gravity accleration
g = 9.81;

syms t;
the_eq = a0 + a1.*t + a2.*t^2 + a3.*t^3;
the_dot_eq = diff(the_eq);
the_2dot_eq = diff(the_dot_eq);
% 
% M = [l2^2*m2+2*l1*l2*m2*cos(the_eq(2,:))+l1^2*(m1+m2) l2^2*m2+l1*l2*m2*cos(the_eq(2,:)); l2^2*m2+l1*l2*m2*cos(the_eq(2,:)) l2^2*m2];
% V = [-m2*l1*l2*sin(the_eq(2,:))*the_dot_eq(2,:)^2-2*m2*l1*l2*sin(the_eq(2,:))*the_dot_eq(1,:)*the_dot_eq(2,:); m2*l1*l2*sin(the_eq(2,:))*the_dot_eq(1,:)^2];
% G = [m2*l2*g*cos(the_eq(1,:)+the_eq(2,:))+(m1+m2)*l1*g*cos(the_eq(1,:)); m2*l2*g*cos(the_eq(1,:)+the_eq(1,:))];
% 
% tau = M*the_2dot_eq + V + G;
% 
% the_2d_r = M^(-1)*(tau - V - G);
% the_d_r = the_2d_r*t;
% the_r = th1 + the_d_r*t; + the_2d_r*0.5*t^2;

Kd = [0.01 1.5];
Kp = [6800 5800];
e = [0;0];
% dtt_r = [0;0];
deltat = 0.001;
tt_r = tht;
dtt_r = tht_dot;
j = 1;

hold off;
for i=0:deltat:1
    
    tt_f = subs(the_eq, t, i);
    dtt_f = subs(the_dot_eq, t, i);
    ddtt_f = subs(the_2dot_eq, t, i);


    T = -Kd*dtt_r + Kp*e;

    M = [l2^2*m2+2*l1*l2*m2*cos(the_eq(2,:))+l1^2*(m1+m2) l2^2*m2+l1*l2*m2*cos(the_eq(2,:)); l2^2*m2+l1*l2*m2*cos(the_eq(2,:)) l2^2*m2];
    V = [-m2*l1*l2*sin(the_eq(2,:))*the_dot_eq(2,:)^2-2*m2*l1*l2*sin(the_eq(2,:))*the_dot_eq(1,:)*the_dot_eq(2,:); m2*l1*l2*sin(the_eq(2,:))*the_dot_eq(1,:)^2];
    G = [m2*l2*g*cos(the_eq(1,:)+the_eq(2,:))+(m1+m2)*l1*g*cos(the_eq(1,:)); m2*l2*g*cos(the_eq(1,:)+the_eq(1,:))];
    
    Minv = M^-1;
    ddtt_r_eq = Minv*(T-V-G);
    ddtt_r = subs(ddtt_r_eq, t, i);
%     ddtt1_r=ddtt(1)
%     ddtt2_r=ddtt(2)
    dtt_r = dtt_r + deltat*ddtt_r;
    tt_r = tt_r + dtt_r*deltat + 0.5*deltat^2*dtt_r;
    e = tt_f - tt_r;

    figure(1)
    plot(i,tt_f(1,:),'*');
    hold on
    plot(i,tt_r(1,:),'red*');
    hold on
    plot(i,e(1,:),'black*')
    figure(2)
    plot(i,tt_f(2,:),'*');
    hold on
    plot(i,tt_r(2,:),'red*');
    hold on
    plot(i,e(2,:),'black*')
    figure(3)
    plot(i,e(2,:),'red*')
    hold on
    plot(i,e(2,:),'*')
 
    j= j + 1;
 
end

