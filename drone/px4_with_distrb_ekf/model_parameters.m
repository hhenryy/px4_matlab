%% UAV Model

% Mass and Inertia
m = 0.66;
Ixx = 1.326e-3;
Iyy = 9.3e-4;
Izz = 1.95e-3;

% Total inertia
I = [Ixx + normrnd(0,0.0001) normrnd(0,0.000001) normrnd(0,0.000001) ; 
    normrnd(0,0.000001) Iyy + normrnd(0,0.00001) normrnd(0,0.000001) ; 
    normrnd(0,0.000001) normrnd(0,0.000001) Izz + normrnd(0,0.0001)];

% Geometry
d = 0.11; % Distance from centre to motor
r_D = 7.997e-3; % Virtual yaw moment arm

% Propulsion:
tau_T = 0.002; % Motor time constant
virtual_controls_mat = [1 1 1 1; -1/sqrt(2) 1/sqrt(2) 1/sqrt(2) -1/sqrt(2); 1/sqrt(2) -1/sqrt(2) 1/sqrt(2) -1/sqrt(2); 1 1 -1 -1]; % Mixin matrix for X Quadrotormixin_mat = virtual_controls_mat'; % mixin matrix
mixin_matrix = virtual_controls_mat'; % mixin matrix
max_total_T_kg = 27.4/9.81; % max thrust of total quadrotor in kg (total), Not just per motor


% Propulsion:
max_total_T = max_total_T_kg * g; % maximum total thrust
max_T = max_total_T / 4; % maximum thrust per motor

hover_perc = m*g / max_total_T; % hover percentage of full throttle
hover_total_T = max_total_T * hover_perc; % total hover thrust
hover_T = hover_total_T / 4; % hover thrust per motor



% Aerodynamic:
C_D = [0.064; 0.067; 0.089].*0; % From Pierro - need to calculate in flight test
wind = [0; 0; 0]; % Wind constants

% EKF Setup
X = [0,0,0,0,0,0,0,0,0];
meas_noise = 0.000001;
process_noise = 0.000001;
dstrb_process_noise = 0.001;



Qk = eye(12)*process_noise;
Rk = eye(9)*meas_noise;
Qk = eye(12)*process_noise;
Qk(10,10)=dstrb_process_noise;
Qk(11,11)=dstrb_process_noise;
Qk(12,12)=dstrb_process_noise;
Rk = eye(9)*meas_noise;

Pk_init = zeros(12);
X_hat_init = zeros(12,1);

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



