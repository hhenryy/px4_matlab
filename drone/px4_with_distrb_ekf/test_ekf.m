

x_hat_prev = zeros(9,1);
u = ones(4,1);
P_prev = zeros(9,9);
yk = ones(9,1);

F = @(X,U)    [1/m*( -m*g*sin( X(7) ) + Fx) - X(3)*X(5) + X(2)*X(6);
               1/m*( m*g*cos( X(7) )*sin( X(8) ) + Fy) - X(1)*X(6) + X(3)*X(4);
               1/m*( m*g*cos( X(7) )*cos( X(8) ) - U(1) - d*U(2) + Fz) + X(2)*X(4) - X(1)*X(5);
               1/Ixx * ( d*U(2) ) - ( (X(5)*X(6))/Ixx )*(Izz - Iyy);
               1/Iyy * ( d*U(3) ) - ( X(4)*X(6)/Iyy)*(Ixx - Izz);
               1/Izz * ( r_D*U(4) ) - (X(4)*X(5)/Izz)*(Izz - Ixx);
               1*X(4) + sin( X(8) )*tan( X(7) )*X(5) + cos( X(8) )*tan( X(7) )*X(6);
               0*X(4) + cos( X(8) )*X(5) + -sin( X(7) )*X(6);
               0*X(4) + sin( X(8) )*sec( X(7) )*X(5) + cos( X(8) )*sec( X(7) )*X(6)
               ];
           
           
           
           
 %% Jacobian
 Fk = @(X,U) [[  0, X(6), -X(5),                    0,                  -X(3),                    X(2),                                                 0,                                                               -g*cos(X(7)), 0]
[ -X(6), 0,  X(4),                    X(3),                   0,                   -X(1),                             g*cos(X(8))*cos(X(7)),                                                      -g*sin(X(8))*sin(X(7)), 0]
[ -X(5), X(4),  0,                    X(2),                  -X(1),                    0,                            -g*cos(X(7))*sin(X(8)),                                                      -g*cos(X(8))*sin(X(7)), 0]
[  0, 0,  0,                    0, (X(6)*(Iyy - Izz))/Ixx,  (X(5)*(Iyy - Izz))/Ixx,                                                 0,                                                                           0, 0]
[  0, 0,  0, -(X(6)*(Ixx - Izz))/Iyy,                   0, -(X(4)*(Ixx - Izz))/Iyy,                                                 0,                                                                           0, 0]
[  0, 0,  0,  (X(5)*(Ixx - Izz))/Izz, (X(4)*(Ixx - Izz))/Izz,                    0,                                                 0,                                                                           0, 0]
[  0, 0,  0,                    1, sin(X(8))*tan(X(7)),  cos(X(8))*tan(X(7)),     X(5)*cos(X(8))*tan(X(7)) - X(6)*sin(X(8))*tan(X(7)),               X(6)*cos(X(8))*(tan(X(7))^2 + 1) + X(5)*sin(X(8))*(tan(X(7))^2 + 1), 0]
[  0, 0,  0,                    0,            cos(X(8)),            -sin(X(8)),                         - X(6)*cos(X(8)) - X(5)*sin(X(8)),                                                                           0, 0]
[  0, 0,  0,                    0, sin(X(8))/cos(X(7)),  cos(X(8))/cos(X(7)), (X(5)*cos(X(8)))/cos(X(7)) - (X(6)*sin(X(8)))/cos(X(7)), (X(6)*cos(X(8))*sin(X(7)))/cos(X(7))^2 + (X(5)*sin(X(8))*sin(X(7)))/cos(X(7))^2, 0]
];

%% Control Update
Pk_ =  Fk(x_hat_prev,u) * P_prev * transpose( Fk(x_hat_prev,u) ) + Qk;
Xk_ = x_hat_prev + F(x_hat_prev,u)*1/250; 
         
Hk = eye(9);
%% Measurement Update
Lk = Pk_*transpose(Hk)*inv(Hk*Pk_*transpose(Hk) + Rk);
Pk = (eye(9) - Lk*Hk)*Pk_;
X_hat = Xk_ + Lk*(yk - Xk_)