clc;clear all;
%%%%%%Input%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
th1 = pi/3;
th2 = pi/3;
th3 = pi/3;
th4 = pi/3;
th5 = pi/3;
th6 = pi/3;	
	
%%%%%%D_H_PARAMETERS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
anpha0=0;
anhpha1=-pi/2;
anpha2=0;
anpha3=-pi/2;
anpha4=pi/2;
anpha5=-pi/2;
a0=0;
a1=0;
a2=0.4318;%
a3=0.0503;
a4=0;
a5=0;
d1=0;
d2=0;
d3=0.1397;%
d4=0.4331;%
d5=0;
d6=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Forward kinematic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
c23=cos(theta2)*cos(theta3)-sin(theta2)*sin(theta3)
s23=cos(theta2)*sin(theta3)+sin(theta2)*cos(theta3);

r11=cos(theta1)*(c23*(cos(theta4)*cos(theta5)*cos(theta6)-sin(theta4)*sin(theta6))-s23*sin(theta5)*cos(theta6))+sin(theta1)*(sin(theta4)*cos(theta5)*cos(theta6)+cos(theta4)*sin(theta6));
r21=sin(theta1)*(c23*(cos(theta4)*cos(theta5)*cos(theta6)-sin(theta4)*sin(theta6))-s23*sin(theta5)*cos(theta6))-cos(theta1)*(sin(theta4)*cos(theta5)*cos(theta6)+cos(theta4)*sin(theta6));
r31=-s23*(cos(theta4)*cos(theta5)*cos(theta6)-sin(theta4)*sin(theta6))-c23*sin(theta5)*cos(theta6);

r12=cos(theta1)*(c23*(-cos(theta4)*cos(theta5)*sin(theta6)-sin(theta4)*cos(theta6))+s23*sin(theta5)*sin(theta6))+sin(theta1)*(cos(theta4)*cos(theta6)-sin(theta4)*cos(theta5)*sin(theta6));
r22=sin(theta1)*(c23*(-cos(theta4)*cos(theta5)*sin(theta6)-sin(theta4)*cos(theta6))+s23*sin(theta5)*sin(theta6))-cos(theta1)*(cos(theta4)*cos(theta6)-sin(theta4)*cos(theta5)*sin(theta6));
r32=-s23*(-cos(theta4)*cos(theta5)*sin(theta6)-sin(theta4)*cos(theta6))+c23*sin(theta5)*sin(theta6);

r13=-cos(theta1)*(c23*cos(theta4)*sin(theta5)+s23*cos(theta5))-sin(theta1)*sin(theta4)*sin(theta5);
r23=-sin(theta1)*(c23*cos(theta4)*sin(theta5)+s23*cos(theta5))+cos(theta1)*sin(theta4)*sin(theta5);
r33=s23*cos(theta4)*sin(theta5)-c23*cos(theta5);

px=cos(theta1)*(a2*cos(theta2)+a3*c23-d4*s23)-d3*sin(theta1);
py=sin(theta1)*(a2*cos(theta2)+a3*c23-d4*s23)+d3*cos(theta1);
pz=-a3*s23-a2*sin(theta2)-d4*c23;

T_0_6=[r11 r12 r13 px;
       r21 r22 r23 py;
       r31 r32 r33 pz;
       0 0 0 1]

T_6_t=[1 0 0 1;
    0 1 0 1;
    0 0 1 2;
    0 0 0 1]
T_0_t=T_0_6*T_6_t

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Forward kinematic (for checking again)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T_0_1=[cos(theta1) -sin(theta1) 0 0;
    sin(theta1) cos(theta1) 0 0;
    0 0 1 0;
    0 0 0 1]

T_1_2=[cos(theta2) -sin(theta2) 0 0;
    0 0 1 0;
    -sin(theta2) -cos(theta2) 0 0;
    0 0 0 1]

T_2_3=[cos(theta3) -sin(theta3) 0 a2;
     sin(theta3) cos(theta3) 0 0;
    0 0 1 d3;
    0 0 0 1]

T_3_4=[cos(theta4) -sin(theta4) 0 a3;
    0 0 1 d4;
    -sin(theta4) -cos(theta4) 0 0;
    0 0 0 1]
T_4_5=[cos(theta5) -sin(theta5) 0 0;
    0 0 -1 0;
    sin(theta5) cos(theta5) 0 0;
    0 0 0 1]
T_5_6=[cos(theta6) -sin(theta6) 0 0;
    0 0 1 0;
    -sin(theta6) -cos(theta6) 0 0;
    0 0 0 1]
T=T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Inverse manipulator kinematics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A=T_0_t*T_6_t^-1
K=((A(1,4)^2+A(2,4)^2+A(3,4)^2)-a2^2-a3^2-d3^2-d4^2)/(2*a2);
%%%%%%%% solutions set #1:%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tt1=atan2(A(2,4),A(1,4))-atan2(d3,-sqrt(A(2,4)^2+A(1,4)^2-d3^2))           %-
tt3=atan2(a3,d4)-atan2(K,sqrt(a3^2+d4^2-K^2))                              %+
tt23=atan2((-a3-a2*cos(tt3))*A(3,4)-(cos(tt1)*A(1,4)+sin(tt1)*A(2,4))*(d4-a2*sin(tt3)),(a2*sin(tt3)-d4)*A(3,4)+(a3+a2*cos(tt3))*(cos(tt1)*A(1,4)+sin(tt1)*A(2,4)))
tt2=tt23-tt3
c4s5=-A(1,3)*cos(tt1)*cos(tt23)-A(2,3)*sin(tt1)*cos(tt23)+A(3,3)*sin(tt23);
s4s5=-A(1,3)*sin(tt1)+A(2,3)*cos(tt1);
 if and(abs(c4s5)<10^-8,abs(s4s5)<10^-8)
     tt5=0   
     tt4=0.1
     s6=-A(1,1)*(cos(tt1)*cos(tt23)*sin(tt4)-sin(tt1)*cos(tt4))-A(2,1)*(sin(tt1)*cos(tt23)*sin(tt4)+cos(tt1)*cos(tt4))+A(3,1)*(sin(tt23)*sin(tt4))
     c6=A(1,1)*((cos(tt1)*cos(tt23)*cos(tt4)+sin(tt1)*sin(tt4))*cos(tt5)-cos(tt1)*sin(tt23)*sin(tt5))+A(2,1)*((sin(tt1)*cos(tt23)*cos(tt4)-cos(tt1)*sin(tt4))*cos(tt5)-sin(tt1)*sin(tt23)*sin(tt5))-A(3,1)*(sin(tt23)*cos(tt4)*cos(tt5)+cos(tt23)*sin(tt5))
     tt6=atan2(s6,c6)
 else
tt4=atan2(s4s5,c4s5)
s5=-A(1,3)*(cos(tt1)*cos(tt23)*cos(tt4)+sin(tt1)*sin(tt4))-A(2,3)*(sin(tt1)*cos(tt23)*cos(tt4)-cos(tt1)*sin(tt4))+A(3,3)*(sin(tt23)*cos(tt4))
c5=A(1,3)*(-cos(tt1)*sin(tt23))+A(2,3)*(-sin(tt1)*sin(tt23))+A(3,3)*(-cos(tt23))
tt5=atan2(s5,c5)
s6=-A(1,1)*(cos(tt1)*cos(tt23)*sin(tt4)-sin(tt1)*cos(tt4))-A(2,1)*(sin(tt1)*cos(tt23)*sin(tt4)+cos(tt1)*cos(tt4))+A(3,1)*(sin(tt23)*sin(tt4))
c6=A(1,1)*((cos(tt1)*cos(tt23)*cos(tt4)+sin(tt1)*sin(tt4))*cos(tt5)-cos(tt1)*sin(tt23)*sin(tt5))+A(2,1)*((sin(tt1)*cos(tt23)*cos(tt4)-cos(tt1)*sin(tt4))*cos(tt5)-sin(tt1)*sin(tt23)*sin(tt5))-A(3,1)*(sin(tt23)*cos(tt4)*cos(tt5)+cos(tt23)*sin(tt5))
tt6=atan2(s6,c6)
end
%%%%%%%%% solutions set #2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tt1_1=atan2(A(2,4),A(1,4))-atan2(d3,sqrt(A(2,4)^2+A(1,4)^2-d3^2))          %+
tt3_1=atan2(a3,d4)-atan2(K,sqrt(a3^2+d4^2-K^2))                            %+
tt23_1=atan2((-a3-a2*cos(tt3_1))*A(3,4)-(cos(tt1_1)*A(1,4)+sin(tt1_1)*A(2,4))*(d4-a2*sin(tt3_1)),(a2*sin(tt3_1)-d4)*A(3,4)+(a3+a2*cos(tt3_1))*(cos(tt1_1)*A(1,4)+sin(tt1_1)*A(2,4)))
tt2_1=tt23_1-tt3_1
c4s5_1=-A(1,3)*cos(tt1_1)*cos(tt23_1)-A(2,3)*sin(tt1_1)*cos(tt23_1)+A(3,3)*sin(tt23_1);
s4s5_1=-A(1,3)*sin(tt1_1)+A(2,3)*cos(tt1_1);
if and(abs(c4s5_1)<10^-8,abs(s4s5_1)<10^-8)
     tt5_1=0   
     tt4_1=0.1
     s6_1=-A(1,1)*(cos(tt1_1)*cos(tt23_1)*sin(tt4_1)-sin(tt1_1)*cos(tt4_1))-A(2,1)*(sin(tt1_1)*cos(tt23_1)*sin(tt4_1)+cos(tt1_1)*cos(tt4_1))+A(3,1)*(sin(tt23_1)*sin(tt4_1))
     c6_1=A(1,1)*((cos(tt1_1)*cos(tt23_1)*cos(tt4_1)+sin(tt1_1)*sin(tt4_1))*cos(tt5_1)-cos(tt1_1)*sin(tt23_1)*sin(tt5_1))+A(2,1)*((sin(tt1_1)*cos(tt23_1)*cos(tt4_1)-cos(tt1_1)*sin(tt4_1))*cos(tt5_1)-sin(tt1_1)*sin(tt23_1)*sin(tt5_1))-A(3,1)*(sin(tt23_1)*cos(tt4_1)*cos(tt5_1)+cos(tt23_1)*sin(tt5_1))
     tt6_1=atan2(s6_1,c6_1)
else
tt4_1=atan2(s4s5_1,c4s5_1)
s5_1=-A(1,3)*(cos(tt1_1)*cos(tt23_1)*cos(tt4_1)+sin(tt1_1)*sin(tt4_1))-A(2,3)*(sin(tt1_1)*cos(tt23_1)*cos(tt4_1)-cos(tt1_1)*sin(tt4_1))+A(3,3)*(sin(tt23_1)*cos(tt4_1))
c5_1=A(1,3)*(-cos(tt1_1)*sin(tt23_1))+A(2,3)*(-sin(tt1_1)*sin(tt23_1))+A(3,3)*(-cos(tt23_1))
tt5_1=atan2(s5_1,c5_1)
s6_1=-A(1,1)*(cos(tt1_1)*cos(tt23_1)*sin(tt4_1)-sin(tt1_1)*cos(tt4_1))-A(2,1)*(sin(tt1_1)*cos(tt23_1)*sin(tt4_1)+cos(tt1_1)*cos(tt4_1))+A(3,1)*(sin(tt23_1)*sin(tt4_1))
c6_1=A(1,1)*((cos(tt1_1)*cos(tt23_1)*cos(tt4_1)+sin(tt1_1)*sin(tt4_1))*cos(tt5_1)-cos(tt1_1)*sin(tt23_1)*sin(tt5_1))+A(2,1)*((sin(tt1_1)*cos(tt23_1)*cos(tt4_1)-cos(tt1_1)*sin(tt4_1))*cos(tt5_1)-sin(tt1_1)*sin(tt23_1)*sin(tt5_1))-A(3,1)*(sin(tt23_1)*cos(tt4_1)*cos(tt5_1)+cos(tt23_1)*sin(tt5_1))
tt6_1=atan2(s6_1,c6_1)
end
%%%%%%%%% solutions set #3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


tt1_2=atan2(A(2,4),A(1,4))-atan2(d3,sqrt(A(2,4)^2+A(1,4)^2-d3^2))          %+
tt3_2=atan2(a3,d4)-atan2(K,-sqrt(a3^2+d4^2-K^2))                           %-
tt23_2=atan2((-a3-a2*cos(tt3_2))*A(3,4)-(cos(tt1_2)*A(1,4)+sin(tt1_2)*A(2,4))*(d4-a2*sin(tt3_2)),(a2*sin(tt3_2)-d4)*A(3,4)+(a3+a2*cos(tt3_2))*(cos(tt1_2)*A(1,4)+sin(tt1_2)*A(2,4)))
tt2_2=tt23_2-tt3_2
c4s5_2=-A(1,3)*cos(tt1_2)*cos(tt23_2)-A(2,3)*sin(tt1_2)*cos(tt23_2)+A(3,3)*sin(tt23_2);
s4s5_2=-A(1,3)*sin(tt1_2)+A(2,3)*cos(tt1_2);
tt4_2=atan2(s4s5_2,c4s5_2)
if and(abs(c4s5_2)<10^-8,abs(s4s5_2)<10^-8)
     tt5_2=0   
     tt4_2=0.1
    s6_2=-A(1,1)*(cos(tt1_2)*cos(tt23_2)*sin(tt4_2)-sin(tt1_2)*cos(tt4_2))-A(2,1)*(sin(tt1_2)*cos(tt23_2)*sin(tt4_2)+cos(tt1_2)*cos(tt4_2))+A(3,1)*(sin(tt23_2)*sin(tt4_2))
    c6_2=A(1,1)*((cos(tt1_2)*cos(tt23_2)*cos(tt4_2)+sin(tt1_2)*sin(tt4_2))*cos(tt5_2)-cos(tt1_2)*sin(tt23_2)*sin(tt5_2))+A(2,1)*((sin(tt1_2)*cos(tt23_2)*cos(tt4_2)-cos(tt1_2)*sin(tt4_2))*cos(tt5_2)-sin(tt1_2)*sin(tt23_2)*sin(tt5_2))-A(3,1)*(sin(tt23_2)*cos(tt4_2)*cos(tt5_2)+cos(tt23_2)*sin(tt5_2))
    tt6_2=atan2(s6_2,c6_2)
else
s5_2=-A(1,3)*(cos(tt1_2)*cos(tt23_2)*cos(tt4_2)+sin(tt1_2)*sin(tt4_2))-A(2,3)*(sin(tt1_2)*cos(tt23_2)*cos(tt4_2)-cos(tt1_2)*sin(tt4_2))+A(3,3)*(sin(tt23_2)*cos(tt4_2))
c5_2=A(1,3)*(-cos(tt1_2)*sin(tt23_2))+A(2,3)*(-sin(tt1_2)*sin(tt23_2))+A(3,3)*(-cos(tt23_2))
tt5_2=atan2(s5_2,c5_2)
s6_2=-A(1,1)*(cos(tt1_2)*cos(tt23_2)*sin(tt4_2)-sin(tt1_2)*cos(tt4_2))-A(2,1)*(sin(tt1_2)*cos(tt23_2)*sin(tt4_2)+cos(tt1_2)*cos(tt4_2))+A(3,1)*(sin(tt23_2)*sin(tt4_2))
c6_2=A(1,1)*((cos(tt1_2)*cos(tt23_2)*cos(tt4_2)+sin(tt1_2)*sin(tt4_2))*cos(tt5_2)-cos(tt1_2)*sin(tt23_2)*sin(tt5_2))+A(2,1)*((sin(tt1_2)*cos(tt23_2)*cos(tt4_2)-cos(tt1_2)*sin(tt4_2))*cos(tt5_2)-sin(tt1_2)*sin(tt23_2)*sin(tt5_2))-A(3,1)*(sin(tt23_2)*cos(tt4_2)*cos(tt5_2)+cos(tt23_2)*sin(tt5_2))
tt6_2=atan2(s6_2,c6_2)
end
%%%%%%%%%  solutions set #4%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tt1_3=atan2(A(2,4),A(1,4))-atan2(d3,-sqrt(A(2,4)^2+A(1,4)^2-d3^2))          %-
tt3_3=atan2(a3,d4)-atan2(K,-sqrt(a3^2+d4^2-K^2))                            %-
tt23_3=atan2((-a3-a2*cos(tt3_3))*A(3,4)-(cos(tt1_3)*A(1,4)+sin(tt1_3)*A(2,4))*(d4-a2*sin(tt3_3)),(a2*sin(tt3_3)-d4)*A(3,4)+(a3+a2*cos(tt3_3))*(cos(tt1_3)*A(1,4)+sin(tt1_3)*A(2,4)))
tt2_3=tt23_3-tt3_3
c4s5_3=-A(1,3)*cos(tt1_3)*cos(tt23_3)-A(2,3)*sin(tt1_3)*cos(tt23_3)+A(3,3)*sin(tt23_3);
s4s5_3=-A(1,3)*sin(tt1_3)+A(2,3)*cos(tt1_3);
if and(abs(c4s5_3)<10^-8,abs(s4s5_3)<10^-8)
     tt5_3=0   
     tt4_3=0.1
     s6_3=-A(1,1)*(cos(tt1_3)*cos(tt23_3)*sin(tt4_3)-sin(tt1_3)*cos(tt4_3))-A(2,1)*(sin(tt1_3)*cos(tt23_3)*sin(tt4_3)+cos(tt1_3)*cos(tt4_3))+A(3,1)*(sin(tt23_3)*sin(tt4_3))
     c6_3=A(1,1)*((cos(tt1_3)*cos(tt23_3)*cos(tt4_3)+sin(tt1_3)*sin(tt4_3))*cos(tt5_3)-cos(tt1_3)*sin(tt23_3)*sin(tt5_3))+A(2,1)*((sin(tt1_3)*cos(tt23_3)*cos(tt4_3)-cos(tt1_3)*sin(tt4_3))*cos(tt5_3)-sin(tt1_3)*sin(tt23_3)*sin(tt5_3))-A(3,1)*(sin(tt23_3)*cos(tt4_3)*cos(tt5_3)+cos(tt23_3)*sin(tt5_3))
     tt6_3=atan2(s6_3,c6_3)
else
    
tt4_3=atan2(s4s5_3,c4s5_3)
s5_3=-A(1,3)*(cos(tt1_3)*cos(tt23_3)*cos(tt4_3)+sin(tt1_3)*sin(tt4_3))-A(2,3)*(sin(tt1_3)*cos(tt23_3)*cos(tt4_3)-cos(tt1_3)*sin(tt4_3))+A(3,3)*(sin(tt23_3)*cos(tt4_3))
c5_3=A(1,3)*(-cos(tt1_3)*sin(tt23_3))+A(2,3)*(-sin(tt1_3)*sin(tt23_3))+A(3,3)*(-cos(tt23_3))
tt5_3=atan2(s5_3,c5_3)
s6_3=-A(1,1)*(cos(tt1_3)*cos(tt23_3)*sin(tt4_3)-sin(tt1_3)*cos(tt4_3))-A(2,1)*(sin(tt1_3)*cos(tt23_3)*sin(tt4_3)+cos(tt1_3)*cos(tt4_3))+A(3,1)*(sin(tt23_3)*sin(tt4_3))
c6_3=A(1,1)*((cos(tt1_3)*cos(tt23_3)*cos(tt4_3)+sin(tt1_3)*sin(tt4_3))*cos(tt5_3)-cos(tt1_3)*sin(tt23_3)*sin(tt5_3))+A(2,1)*((sin(tt1_3)*cos(tt23_3)*cos(tt4_3)-cos(tt1_3)*sin(tt4_3))*cos(tt5_3)-sin(tt1_3)*sin(tt23_3)*sin(tt5_3))-A(3,1)*(sin(tt23_3)*cos(tt4_3)*cos(tt5_3)+cos(tt23_3)*sin(tt5_3))
tt6_3=atan2(s6_3,c6_3)
end
%%%%%%%% solutions set #5:%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tt1_4=tt1    
tt3_4=tt3                      
tt2_4=tt2
tt4_4=tt4+pi
tt5_4=-tt5
tt6_4=tt6+pi
%%%%%%%% solutions set #6:%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tt1_5=tt1_1    
tt3_5=tt3_1                      
tt2_5=tt2_1
tt4_5=tt4_1+pi
tt5_5=-tt5_1
tt6_5=tt6_1+pi
%%%%%%%% solutions set #7:%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tt1_6=tt1_2    
tt3_6=tt3_2                      
tt2_6=tt2_2
tt4_6=tt4_2+pi
tt5_6=-tt5_2
tt6_6=tt6_2+pi
%%%%%%%% solutions set #8:%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tt1_7=tt1_3    
tt3_7=tt3_3                      
tt2_7=tt2_3
tt4_7=tt4_3+pi
tt5_7=-tt5_3
tt6_7=tt6_3+pi
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%LE PHU NGUYEN -I.R.S.LAB Room 7-417 
%%%University of Ulsan






















