%% Determine Noise matrices for EKF
%% Setup
% Add subfolders to path
addpath(genpath('simulation_setup'));
setup_simulation;

model_parameters;

X = [0,0,0,0,0,0,0,0,0];
meas_noise = 0.0001;
process_noise = 0.00001;



Qk = eye(9)*process_noise;
Rk = eye(9)*process_noise;


Pk_init = zeros(9);
X_hat_init = zeros(9,1);

X = X_hat_init;


Fx = 0; 
Fy = 0;
Fz = 0;




At = [
    [  0,  X(6), -X(5),                    0,                  -X(3),                    X(2),                                                 0,                                                               -g*cos(X(8)), 0]
[ -X(6),  0,  X(4),                    X(3),                   0,                   -X(1),                             g*cos(X(7))*cos(X(8)),                                                      -g*sin(X(7))*sin(X(8)), 0]
[  X(5), -X(4),  0,                   -X(2),                   X(1),                    0,                            -g*cos(X(8))*sin(X(7)),                                                      -g*cos(X(7))*sin(X(8)), 0]
[  0,  0,  0,                    0, (X(6)*(Iyy - Izz))/Ixx,  (X(5)*(Iyy - Izz))/Ixx,                                                 0,                                                                           0, 0]
[  0,  0,  0, -(X(6)*(Ixx - Izz))/Iyy,                   0, -(X(4)*(Ixx - Izz))/Iyy,                                                 0,                                                                           0, 0]
[  0,  0,  0,  (X(5)*(Ixx - Izz))/Izz, (X(4)*(Ixx - Izz))/Izz,                    0,                                                 0,                                                                           0, 0]
[  0,  0,  0,                    1, sin(X(7))*tan(X(8)),  cos(X(7))*tan(X(8)),     X(5)*cos(X(7))*tan(X(8)) - X(6)*sin(X(7))*tan(X(8)),               X(6)*cos(X(7))*(tan(X(8))^2 + 1) + X(5)*sin(X(7))*(tan(X(8))^2 + 1), 0]
[  0,  0,  0,                    0,            cos(X(7)),            -sin(X(7)),                         - X(6)*cos(X(7)) - X(5)*sin(X(7)),                                                                           0, 0]
[  0,  0,  0,                    0, sin(X(7))/cos(X(8)),  cos(X(7))/cos(X(8)), (X(5)*cos(X(7)))/cos(X(8)) - (X(6)*sin(X(7)))/cos(X(8)), (X(6)*cos(X(7))*sin(X(8)))/cos(X(8))^2 + (X(5)*sin(X(7))*sin(X(8)))/cos(X(8))^2, 0]
];
 
lam = [-At Qk;zeros(9), At' ];

C_noise = expm(lam*(1/sim_freq));
[col,row] =size(C_noise);

C11 = C_noise(1:col/2,1:row/2);
C12 = C_noise(1:col/2,row/2+1:end);
C21 = C_noise(col/2+1:end,1:row/2);
C22 = C_noise(col/2+1:end,row/2+1:end);
Qk = C22'*C12
