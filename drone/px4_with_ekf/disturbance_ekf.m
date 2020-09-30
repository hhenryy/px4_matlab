%% Playground for Simulink Model EKF

%F(X,U) = [
% X: states
%1 = U,
%2 = V,
%3 = W,
%4 = P,
%5 = Q,
%6 = R,
%7 = theta,
%8 = phi,
%9 = psi,
%           ]'
% U: control inputs
%1 = del_t 
%2 = del_a
%3 = del_e
%4 = del_r
F = @(X,U)    [1/m*( -m*g*sin( X(7) ) + Fx) - X(3)*X(5) + X(2)*X(6);
               1/m*( m*g*cos( X(7) )*sin( X(8) ) + Fy) - X(1)*X(6) + X(3)*X(4);
               1/m*( m*g*cos( X(7) )*cos( X(8) ) - U(1) - d*U(2) + Fz) + X(2)*X(4) - X(1)*X(5);
               1/Ix * ( d*U(2) ) - ( (X(5)*X(6))/Ix )*(Iz - Iy);
               1/Iy * ( d*U(3) ) - ( X(4)*X(6)/Iy)*(Ix - Iz);
               1/Iz * ( rd/Rld*U(4) ) - (X(4)*X(5)/Iz)*(Iz - Ix);
               1*X(4) + sin( X(8) )*tan( X(7) )*X(5) + cos( X(8) )*tan( X(7) )*X(6);
               0*X(4) + cos( X(8) )*X(5) + -sin( X(7) )*X(6);
               0*X(4) + sin( X(8) )*sec( X(7) )*X(5) + cos( X(8) )*sec( X(7) )*X(6)
               ];
           
           
 %% Jacobian
 Fk = @(X,U) [[ 0, X(6), -X(5), 0, -X(3), X(2), 0, -g*cos( X(7) ), 0];
             [ -X(6), 0, X(4), X(3), 0, -X(1), g*cos( x(8) )*cos( X(7) ), -g*sin( X(8) )*sin( X(7) ), 0];
             [ -X(5), X(4), 0, X(2), -X(1), 0, -g*cos( X(7) )*sin( X(8) ), -g*cos( X(8) )*sin( X(7) ), 0];
             [ 0, 0, 0, -( X(6)*(Ix - Iz))/Iy, 0, -( X(4)*(Ix - Iz))/Iy, 0, 0, 0];
             [ 0, 0, 0, ( X(5)*(Ix - Iz))/Iz, ( X(4)*(Ix - Iz))/Iz, 0, 0, 0, 0];
             [ 0, 0, 0, 1, sin( X(8) )*tan( X(7) ), cos( X(8) )*tan( X(7) ), X(5)*cos( X(8) )*tan( X(7) ) - X(6)*sin( X(8) )*tan( X(7) ), X(6)*cos( X(8) )*(tan( X(7) )^2 + 1) + X(5)*sin( X(8) )*(tan( X(7) )^2 + 1), 0]];

%% Control Update
Pk_ =  Fk(x_hat_prev,u) * P_prev * transpose( Fk(x_hat_prev,u) ) + Qk;
Xk_ = x_hat_prev + Fk(x_hat_prev,u)*1/250; 
         
Hk = eye(6);
%% Measurement Update
Lk = Pk_*transpose(Hk)*inv(Hk*Pk_*transpose(Hk) + Rk);
Pk = (eye(6) - Lk*Hk)*Pk_;
X_hat = Xk_ + Lk*(yk - [ H1(Xk_);H2(Xk_)]);
 
 
 
 
 
 
 
 