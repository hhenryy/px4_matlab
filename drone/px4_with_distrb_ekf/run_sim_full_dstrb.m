%% Simulation
% Duration of simulation
sim_time = 30;
% Step inputs
pos_e_step = [0,0,0];
yaw_step = 0.00;

%% Setup
% Add subfolders to path
addpath(genpath('simulation_setup'));
setup_simulation;

%% Sensor Noise
sensor_noise = 0;

%% DISTURBANCE SELECTOR
% Step
step_mag = 0.1; % second
step_delay = 1; % second

% Pulse 
pulse_amplitude = 0.1;
pulse_period = 40; % seconds

% Sinusoid
sinus_amplitude = 0.1;
sinus_bias = 0;
sinus_freq = 10;

% Ramp 
ramp_slope = 0.01;
ramp_max = 0.1;
ramp_start = 1;

%% EKF INIT
meas_noise = 0.000001;
process_noise = 0.000001;
dstrb_process_noise = 0.001;



Qk = eye(15)*process_noise;
Rk = eye(9)*meas_noise;
Qk = eye(15)*process_noise;
Qk(10,10)=dstrb_process_noise;
Qk(11,11)=dstrb_process_noise;
Qk(12,12)=dstrb_process_noise;
Qk(13,13)=dstrb_process_noise;
Qk(14,14)=dstrb_process_noise;
Qk(15,15)=dstrb_process_noise;
Rk = eye(9)*meas_noise;

Pk_init = zeros(15);
X_hat_init = zeros(15,1);

X = X_hat_init;



P=0;
Q=0;
R=0;
U=0;
V=0;
W=0;
phi=0;
theta=0;
psi=0;


At = [
[  0,  R, -Q,                    0,                  -W,                    V,                                                 0,                                                               -g*cos(theta), 0, 1/m,   0,   0,     0,     0,     0]
[ -R,  0,  P,                    W,                   0,                   -U,                             g*cos(phi)*cos(theta),                                                      -g*sin(phi)*sin(theta), 0,   0, 1/m,   0,     0,     0,     0]
[  Q, -P,  0,                   -V,                   U,                    0,                            -g*cos(theta)*sin(phi),                                                      -g*cos(phi)*sin(theta), 0,   0,   0, 1/m,     0,     0,     0]
[  0,  0,  0,                    0, (R*(Iyy - Izz))/Ixx,  (Q*(Iyy - Izz))/Ixx,                                                 0,                                                                           0, 0,   0,   0,   0, 1/Ixx,     0,     0]
[  0,  0,  0, -(R*(Ixx - Izz))/Iyy,                   0, -(P*(Ixx - Izz))/Iyy,                                                 0,                                                                           0, 0,   0,   0,   0,     0, 1/Iyy,     0]
[  0,  0,  0,  (Q*(Ixx - Izz))/Izz, (P*(Ixx - Izz))/Izz,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0, 1/Izz]
[  0,  0,  0,                    1, sin(phi)*tan(theta),  cos(phi)*tan(theta),     Q*cos(phi)*tan(theta) - R*sin(phi)*tan(theta),               R*cos(phi)*(tan(theta)^2 + 1) + Q*sin(phi)*(tan(theta)^2 + 1), 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,            cos(phi),            -sin(phi),                         - R*cos(phi) - Q*sin(phi),                                                                           0, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0, sin(phi)/cos(theta),  cos(phi)/cos(theta), (Q*cos(phi))/cos(theta) - (R*sin(phi))/cos(theta), (R*cos(phi)*sin(theta))/cos(theta)^2 + (Q*sin(phi)*sin(theta))/cos(theta)^2, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0,     0]
];
 
lam = [-At Qk;zeros(15), At' ];

C_noise = expm(lam*(1/sim_freq));
[col,row] =size(C_noise);

C11 = C_noise(1:col/2,1:row/2);
C12 = C_noise(1:col/2,row/2+1:end);
C21 = C_noise(col/2+1:end,1:row/2);
C22 = C_noise(col/2+1:end,row/2+1:end);
Qk = C22'*C12;


%% Run simulation
%Ssim('px4_mc_sim_full_distrb_ekf');