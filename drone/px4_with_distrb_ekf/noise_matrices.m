%% Determine Noise matrices for EKF
%% Setup
% Add subfolders to path
addpath(genpath('simulation_setup'));
setup_simulation;

model_parameters;

X = [0,0,0,0,0,0,0,0,0,0,0,0];
meas_noise = 0.0001;
process_noise = 0.00001;
dstrb_process_noise = 0.001;
U=X(1);
V=X(2);
W=X(3);
P=X(4);
Q=X(5);
R=X(6);
phi=X(7);
theta=X(8);
psi=X(9);
Fx=X(10);
Fy=X(11);
Fz=X(12);


Qk = eye(12)*process_noise;
Qk(10,10)=dstrb_process_noise;
Qk(11,11)=dstrb_process_noise;
Qk(12,12)=dstrb_process_noise;
Rk = eye(9)*meas_noise;


Pk_init = zeros(12,12);
X_hat_init = zeros(12,1);



At = [
[  0,  R, -Q,                    0,                  -W,                    V,                                                 0,                                                               -g*cos(theta), 0, 1/m,   0,   0]
[ -R,  0,  P,                    W,                   0,                   -U,                             g*cos(phi)*cos(theta),                                                      -g*sin(phi)*sin(theta), 0,   0, 1/m,   0]
[  Q, -P,  0,                   -V,                   U,                    0,                            -g*cos(theta)*sin(phi),                                                      -g*cos(phi)*sin(theta), 0,   0,   0, 1/m]
[  0,  0,  0,                    0, (R*(Iyy - Izz))/Ixx,  (Q*(Iyy - Izz))/Ixx,                                                 0,                                                                           0, 0,   0,   0,   0]
[  0,  0,  0, -(R*(Ixx - Izz))/Iyy,                   0, -(P*(Ixx - Izz))/Iyy,                                                 0,                                                                           0, 0,   0,   0,   0]
[  0,  0,  0,  (Q*(Ixx - Izz))/Izz, (P*(Ixx - Izz))/Izz,                    0,                                                 0,                                                                           0, 0,   0,   0,   0]
[  0,  0,  0,                    1, sin(phi)*tan(theta),  cos(phi)*tan(theta),     Q*cos(phi)*tan(theta) - R*sin(phi)*tan(theta),               R*cos(phi)*(tan(theta)^2 + 1) + Q*sin(phi)*(tan(theta)^2 + 1), 0,   0,   0,   0]
[  0,  0,  0,                    0,            cos(phi),            -sin(phi),                         - R*cos(phi) - Q*sin(phi),                                                                           0, 0,   0,   0,   0]
[  0,  0,  0,                    0, sin(phi)/cos(theta),  cos(phi)/cos(theta), (Q*cos(phi))/cos(theta) - (R*sin(phi))/cos(theta), (R*cos(phi)*sin(theta))/cos(theta)^2 + (Q*sin(phi)*sin(theta))/cos(theta)^2, 0,   0,   0,   0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0]
];
 
lam = [-At Qk;zeros(12), At' ];

C_noise = expm(lam*(1/sim_freq));
[col,row] =size(C_noise);

C11 = C_noise(1:col/2,1:row/2);
C12 = C_noise(1:col/2,row/2+1:end);
C21 = C_noise(col/2+1:end,1:row/2);
C22 = C_noise(col/2+1:end,row/2+1:end);
Qk = C22'*C12;
