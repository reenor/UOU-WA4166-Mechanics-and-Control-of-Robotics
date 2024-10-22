function [R, P] = DH_2_R_P(alpha, a, d, theta)

    R = [ cosd(theta)     -sind(theta)*cosd(alpha)    sind(theta)*sind(alpha);
          sind(theta)      cosd(theta)*cosd(alpha)   -cosd(theta)*sind(alpha);
          0                            sind(alpha)                cosd(alpha)  ];
    P = [ a*cosd(theta);
          a*sind(theta);
          d              ];

end