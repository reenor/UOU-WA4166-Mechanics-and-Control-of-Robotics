clc
clear all
close all
syms time 
m1=10;
m2=5;
l=0.5;
tt01=30*pi/180;
tt02=150*pi/180;
ttf1=150*pi/180;
ttf2=30*pi/180; 


a01=pi/6;
a02=5*pi/6;
a21=pi;
a22=-pi;
a31=-4*pi/3;
a32=4*pi/3;
g=9.81;
i=1;
deltat=0.01;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%              P_D Controller                        %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tt1_r=30*pi/180;
% tt2_r=150*pi/180;
% dtt1_r=0;
% dtt2_r=0;
% e1=0;
% e2=0;
% 
% Kp1=9000;
% Kp2=7800;
% Kv1=0.1;
% Kv2=2.5;
% hold off;
% for t=0:deltat:1
%     tt1=a01+a21*t*t+a31*t^3;
%     tt2=a02+a22*t*t+a32*t^3;
%     dtt1=2*a21*t+3*a31*t^2;
%     dtt2=2*a22*t+3*a32*t^2;
%     ddtt1=2*a21+6*a31*t;
%     ddtt2=2*a22+6*a32*t;
% 
%     tque1=-Kv1*dtt1_r+Kp1*e1;
%     tque2=-Kv2*dtt2_r+Kp2*e2;
% 
% 
%     T=[tque1;tque2];
%     M=[l^2*m2+2*l^2*m2*cos(tt2_r)+l^2*(m1+m2) l^2*m2+l^2*m2*cos(tt2_r);l^2*m2+l^2*m2*cos(tt2_r) l^2*m2];
%     V=[-m2*l^2*sin(tt2_r)*dtt2_r^2-2*m2*l^2*sin(tt2_r)*dtt1_r*dtt2_r;m2*l^2*sin(tt2_r)*dtt1^2];
%     G=[m2*l*g*cos(tt1_r+tt2_r)+(m1+m2)*l*g*cos(tt1_r);m2*l*g*cos(tt1_r+tt2_r)];
%     Minv=M^-1;
%     
%     ddtt=Minv*(T-V-G)
%     ddtt1_r=ddtt(1)
%     ddtt2_r=ddtt(2)
%     dtt1_r=dtt1_r+deltat*ddtt1_r;
%     dtt2_r=dtt2_r+deltat*ddtt2_r;
%     tt1_r=tt1_r+dtt1_r*deltat+0.5*deltat^2*ddtt1_r;
%     tt2_r=tt2_r+dtt2_r*deltat+0.5*deltat^2*ddtt2_r;
%     e1=tt1-tt1_r;
%     e2=tt2-tt2_r;
% 
%     figure(1)
%     plot(t,tt1,'blue*');
%     hold on
%     plot(t,tt1_r,'red*');
%     hold on
%     plot(t,e1,'black*')
%     legend({'Desired', 'Real', 'Error'}, 'FontSize', 12);
%     title('PD Joint#1');
%   
%     figure(2)
%     plot(t,tt2,'blue*');
%     hold on
%     plot(t,tt2_r,'red*');
%     hold on
%     plot(t,e2,'black*');
%     legend({'Desired', 'Real', 'Error'}, 'FontSize', 12);
%     title('PD Joint#2');
%   
%     figure(3)
%     plot(t,e1,'red*')
%     hold on
%     plot(t,e2,'blue*');
%     legend({'Error#1', 'Error#2'}, 'FontSize', 12);
%     title('PD Error');
%  
%     i=i+1;
% end


%&&&&&& &&&&&&&&&&&&&&&&&&&&%&&&&&&&&&&&&&&&&&&&&&&&&&&%&&&&&&&&&&&&&&&&&&&
%&&&&&&&&&         P_D And                                  &&%&&&&&&&&&&&&&&&&&&&
%&&&&&&&&&                                           &&&&&&&&&&%&&&&&&&&&&&
%&&&&&&&&&&&&&&&&&&&&&&&&&&%&&&&&&&&&&&&&&&&&&&&&&&&&&%&&&&&&&&&&&&&&&&&&&&
tt1_r=30*pi/180;
tt2_r=150*pi/180;
dtt1_r=0;
dtt2_r=0;
e1=0;
e2=0;
Kp1=9000;
Kp2=7800;
Kv1=0.1;
Kv2=2.5;
for t=0:deltat:1
    tt1=a01+a21*t*t+a31*t^3;
    tt2=a02+a22*t*t+a32*t^3;
    dtt1=2*a21*t+3*a31*t^2;
    dtt2=2*a22*t+3*a32*t^2;
    ddtt1=2*a21+6*a31*t;
    ddtt2=2*a22+6*a32*t;
    g1=m2*l*g*cos(tt1+tt2)+(m1+m2)*l*g*cos(tt1);
    g2=m2*l*g*cos(tt1+tt2);


    tque1=g1-Kv1*dtt1_r+Kp1*e1;
    tque2=g2-Kv2*dtt2_r+Kp2*e2;




    T=[tque1;tque2];
    M=[l^2*m2+2*l^2*m2*cos(tt2_r)+l^2*(m1+m2) l^2*m2+l^2*m2*cos(tt2_r);l^2*m2+l^2*m2*cos(tt2_r) l^2*m2];
    V=[-m2*l^2*sin(tt2_r)*dtt2_r^2-2*m2*l^2*sin(tt2_r)*dtt1_r*dtt2_r;m2*l^2*sin(tt2_r)*dtt1^2];
    G=[m2*l*g*cos(tt1_r+tt2_r)+(m1+m2)*l*g*cos(tt1_r);m2*l*g*cos(tt1_r+tt2_r)];
    Minv=M^-1;
    ddtt=Minv*(T-V-G)
    ddtt1_r=ddtt(1)
    ddtt2_r=ddtt(2)
    dtt1_r=dtt1_r+deltat*ddtt1_r;
    dtt2_r=dtt2_r+deltat*ddtt2_r;
    tt1_r=tt1_r+dtt1_r*deltat+0.5*deltat^2*ddtt1_r;
    tt2_r=tt2_r+dtt2_r*deltat+0.5*deltat^2*ddtt2_r;
    e1=tt1-tt1_r;
    e2=tt2-tt2_r;

    figure(4)
    plot(t,tt1,'blue*');
    hold on
    plot(t,tt1_r,'red*');
    hold on
    plot(t,e1,'black*')
    legend({'Desired', 'Real', 'Error'}, 'FontSize', 12);
    title('PD + Gravity (Joint#1)');

    figure(5)
    plot(t,tt2,'blue*');
    hold on
    plot(t,tt2_r,'red*');
    hold on
    plot(t,e2,'black*')
    legend({'Desired', 'Real', 'Error'}, 'FontSize', 12);
    title('PD + Gravity (Joint#2)');
    
    figure(6)
    plot(t,e1,'red*')
    hold on
    plot(t,e2,'blue*')
    hold on
    legend({'Error#1', 'Error#2'}, 'FontSize', 12);
    title('PD + Gravity (Error)');
    
    i=i+1;
end
% &&&&&&&&&&&&&&&&&&&&&&&&&&%&&&&&&&&&&&&&&&&&&&&&&&&&&%&&&&&&&&&&&&&&&&&&&&
% &&&&&&&&&&&&&&&&&&&&&&&&&&%&&&&&&&&&&&&&&&&&&&&&&&&&&%&&&&&&&&&&&&&&&&&&&&
% &&&&&&&&&&&&&&&&&&&&&&&&&&%&&&&&&&&&&&&&&&&&&&&&&&&&&%&&&&&&&&&&&&&&&&&&&&
% &&&&&&&&&&&&&&&&&&&&&&&&&&%&&&&&&&&&&&&&&&&&&&&&&&&&&%&&&&&&&&&&&&&&&&&&&&
% tt1_r=30*pi/180;
% tt2_r=150*pi/180;
% dtt1_r=0;
% dtt2_r=0;
% e1=0;
% e2=0;
% de1=0;
% de2=0;
% Kp1=7800;
% Kp2=5800;
% Kv1=0.1;
% Kv2=0.5;
% 
% for t=0:deltat:1
%     tt1=a01+a21*t*t+a31*t^3;
%     tt2=a02+a22*t*t+a32*t^3;
%     dtt1=2*a21*t+3*a31*t^2;
%     dtt2=2*a22*t+3*a32*t^2;
%     ddtt1=2*a21+6*a31*t;
%     ddtt2=2*a22+6*a32*t;
% 
%     tque_c1=ddtt1+Kp1*e1+Kv1*de1
%     tque_c2=ddtt2+Kp2*e2+Kv2*de2
% 
%     alpha=[l^2*m2+2*l^2*m2*cos(tt2)+l^2*(m1+m2) l^2*m2+l^2*m2*cos(tt2);l^2*m2+l^2*m2*cos(tt2) l^2*m2];
%     M=alpha;
% 
% 
%     V=[-m2*l^2*sin(tt2_r)*dtt2_r^2-2*m2*l^2*sin(tt2_r)*dtt1_r*dtt2_r;m2*l^2*sin(tt2_r)*dtt1^2];
%     G=[m2*l*g*cos(tt1_r+tt2_r)+(m1+m2)*l*g*cos(tt1_r);m2*l*g*cos(tt1_r+tt2_r)];
%     beta=V+G;
%     T=alpha*[tque_c1;tque_c2]+beta
%     ddtt=M\(T-V-G);
% 
%     ddtt1_r=ddtt(1);
%     ddtt2_r=ddtt(2);
%     dtt1_r=dtt1_r+deltat*ddtt1_r;
%     dtt2_r=dtt2_r+deltat*ddtt2_r;
%     tt1_r=tt1_r+dtt1_r*deltat+0.5*deltat^2*ddtt1_r;
%     tt2_r=tt2_r+dtt2_r*deltat+0.5*deltat^2*ddtt2_r;
%     e1=tt1-tt1_r;
%     e2=tt2-tt2_r;
%     de1=dtt1-dtt1_r;
%     de2=dtt2-dtt2_r;
% 
%     figure(7)
%     plot(t,tt1,'blue*');
%     hold on
%     plot(t,tt1_r,'red*');
%     hold on
%     plot(t,e1,'black*')
%     legend({'Desired', 'Real', 'Error'}, 'FontSize', 12);
%     title('Torque Control (Joint#1)');
% 
%     figure(8)
%     plot(t,tt2,'blue*');
%     hold on
%     plot(t,tt2_r,'red*');
%     hold on
%     plot(t,e2,'black*')
%     legend({'Desired', 'Real', 'Error'}, 'FontSize', 12);
%     title('Torque Control (Joint#2)');
%     
%     figure(9)
%     plot(t,e1,'red*')
%     hold on
%     plot(t,e2,'blue*')
%     legend({'Error#1', 'Error#2'}, 'FontSize', 12);
%     title('Torque Control (Error)');
%  
%     i=i+1;
% end



































%%%%%check the tr?ectory:%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for  t=0:deltat:0.5
%     P_0_2=[cos(tt(1,i)+tt(2,i)) -sin(tt(1,i)+tt(2,i)) 0 0.5*cos(tt(1,i));
%            sin(tt(1,i)+tt(2,i)) cos(tt(1,i)+tt(2,i)) 0 0.5*sin(tt(1,i));
%            0 0 1 0;
%            0 0 0 1];
%    T_2=[0.5 0 0 1]';
%    T_0=P_0_2*T_2;
%      figure(5)
%      plot(T_0(1,1),T_0(2,1),'*')%end efector trajectory
%        hold on
%        
%        
% tt1=a01+a21*t*t+a31*t^3;
% tt2=a02+a22*t*t+a32*t^3     ;
%   P_0_2_d=[cos(tt1+tt2) -sin(tt1+tt2) 0 0.5*cos(tt1);
%            sin(tt1+tt2) cos(tt1+tt2) 0 0.5*sin(tt1);
%            0 0 1 0;
%            0 0 0 1];     
%      T_0_d=P_0_2_d*T_2;
%        plot(T_0_d(1,1),T_0_d(2,1),'*red')%end efector trajectory 
%      i=i+1;
% end
% %%%%%Jacobian matrix:%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%