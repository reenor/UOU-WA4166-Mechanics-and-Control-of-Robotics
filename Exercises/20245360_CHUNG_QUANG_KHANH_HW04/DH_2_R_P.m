function [R, P] = DH_2_R_P(alpha, a, d, theta)

    R = [ cosd(theta)               -sind(theta)                0;
          sind(theta)*cosd(alpha)    cosd(theta)*cosd(alpha)   -sind(alpha);
          sind(theta)*sind(alpha)    cosd(theta)*sind(alpha)    cosd(alpha)  ];

    P = [  a;
          -d*sind(alpha);
           d*cosd(alpha)  ];

end