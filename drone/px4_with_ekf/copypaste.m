

%1 = X(1),
%2 = X(2),
%3 = X(3),
%4 = X(4),
%5 = X(5),
%6 = X(6),
%7 = X(7)
%8 = X(8),
%9 = psi,


[      1,  X(6)/250, -X(5)/250,                          0,                    -X(3)/250,                      X(2)/250,                                                             0,                                                                     -(g*cos(X(8)))/250, 0]
[ -X(6)/250,      1,  X(4)/250,                      X(3)/250,                         0,                     -X(1)/250,                                   (g*cos(X(7))*cos(X(8)))/250,                                                            -(g*sin(X(7))*sin(X(8)))/250, 0]
[  X(5)/250, -X(4)/250,      1,                     -X(2)/250,                     X(1)/250,                          0,                                  -(g*cos(X(8))*sin(X(7)))/250,                                                            -(g*cos(X(7))*sin(X(8)))/250, 0]
[      0,      0,      0,                          1, (X(6)*(Iyy - Izz))/(250*Ixx),  (X(5)*(Iyy - Izz))/(250*Ixx),                                                             0,                                                                                       0, 0]
[      0,      0,      0, -(X(6)*(Ixx - Izz))/(250*Iyy),                         1, -(X(4)*(Ixx - Izz))/(250*Iyy),                                                             0,                                                                                       0, 0]
[      0,      0,      0,  (X(5)*(Ixx - Izz))/(250*Izz), (X(4)*(Ixx - Izz))/(250*Izz),                          1,                                                             0,                                                                                       0, 0]
[      0,      0,      0,                      1/250, (sin(X(7))*tan(X(8)))/250,  (cos(X(7))*tan(X(8)))/250, (X(5)*cos(X(7))*tan(X(8)))/250 - (X(6)*sin(X(7))*tan(X(8)))/250 + 1,               (X(6)*cos(X(7))*(tan(X(8))^2 + 1))/250 + (X(5)*sin(X(7))*(tan(X(8))^2 + 1))/250, 0]
[      0,      0,      0,                          0,              cos(X(7))/250,              -sin(X(7))/250,                         - (X(6)*cos(X(7)))/250 - (X(5)*sin(X(7)))/250,                                                                                       1, 0]
[      0,      0,      0,                          0, sin(X(7))/(250*cos(X(8))),  cos(X(7))/(250*cos(X(8))), (X(5)*cos(X(7)))/(250*cos(X(8))) - (X(6)*sin(X(7)))/(250*cos(X(8))), (X(6)*cos(X(7))*sin(X(8)))/(250*cos(X(8))^2) + (X(5)*sin(X(7))*sin(X(8)))/(250*cos(X(8))^2), 1]
