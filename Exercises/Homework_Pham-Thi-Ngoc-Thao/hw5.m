clear all;
clc;

th1 = 30;
th_dot1 = 0;
th2 = 150;
th_dot2 = 0;

thef1 = 150;
thef_dot1 = 0;
thef2 = 30;
thef_dot2 = 0;

l1 = 0.5;
l2 = 0.5;
m1 = 10;
m2 = 5;

tf = 1;
t = 0;

tau1 = 10*sin(0.5*pi*t);
tau2 = 10*sin(0.5*pi*t);
tau = [tau1; tau2];

M = [l2^2*m2+2*l1*l2*m2*cosd(th2)+l1^2*(m1+m2) l2^2*m2+l1*l2*m2*cosd(th2); l2^2*m2+l1*l2*m2*cosd(th2) l2^2*m2];
V = [-m2*l1*l2*sind(th2)*th_dot2^2-2*m2*l1*l2*sind(th2)*th_dot1*th_dot2; m2*l1*l2*sind(th2)*th_dot1^2];
G = [m2*l2*9.81*cosd(th1+th2)+(m1+m2)*l1*9.81*cosd(th1); m2*l2*9.81*cosd(th1+th2)];

th_2dot = M^(-1)*(tau - V -G);

