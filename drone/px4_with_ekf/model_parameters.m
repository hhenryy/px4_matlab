%% UAV Model

% Mass and Inertia
m = 0.66;
Ixx = 1.326e-3;
Iyy = 9.3e-4;
Izz = 1.95e-3;

% Total inertia
I = [Ixx 0 0 ; 0 Iyy 0 ; 0 0 Izz];

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



Qk = eye(9)*process_noise;
Rk = eye(9)*process_noise;


Pk_init = zeros(9);
X_hat_init = zeros(9,1);

X = X_hat_init;


Fx = 0; 
Fy = 0;
Fz = 0;

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
[      1,  R/250, -Q/250,                          0,                    -W/250,                      V/250,                                                             0,                                                                     -(g*cos(theta))/250, 0]
[ -R/250,      1,  P/250,                      W/250,                         0,                     -U/250,                                   (g*cos(phi)*cos(theta))/250,                                                            -(g*sin(phi)*sin(theta))/250, 0]
[  Q/250, -P/250,      1,                     -V/250,                     U/250,                          0,                                  -(g*cos(theta)*sin(phi))/250,                                                            -(g*cos(phi)*sin(theta))/250, 0]
[      0,      0,      0,                          1, (R*(Iyy - Izz))/(250*Ixx),  (Q*(Iyy - Izz))/(250*Ixx),                                                             0,                                                                                       0, 0]
[      0,      0,      0, -(R*(Ixx - Izz))/(250*Iyy),                         1, -(P*(Ixx - Izz))/(250*Iyy),                                                             0,                                                                                       0, 0]
[      0,      0,      0,  (Q*(Ixx - Izz))/(250*Izz), (P*(Ixx - Izz))/(250*Izz),                          1,                                                             0,                                                                                       0, 0]
[      0,      0,      0,                      1/250, (sin(phi)*tan(theta))/250,  (cos(phi)*tan(theta))/250, (Q*cos(phi)*tan(theta))/250 - (R*sin(phi)*tan(theta))/250 + 1,               (R*cos(phi)*(tan(theta)^2 + 1))/250 + (Q*sin(phi)*(tan(theta)^2 + 1))/250, 0]
[      0,      0,      0,                          0,              cos(phi)/250,              -sin(phi)/250,                         - (R*cos(phi))/250 - (Q*sin(phi))/250,                                                                                       1, 0]
[      0,      0,      0,                          0, sin(phi)/(250*cos(theta)),  cos(phi)/(250*cos(theta)), (Q*cos(phi))/(250*cos(theta)) - (R*sin(phi))/(250*cos(theta)), (R*cos(phi)*sin(theta))/(250*cos(theta)^2) + (Q*sin(phi)*sin(theta))/(250*cos(theta)^2), 1]
];
 
lam = [-At Qk;zeros(9), At' ];

C_noise = expm(lam*(1/sim_freq));
[col,row] =size(C_noise);

C11 = C_noise(1:col/2,1:row/2);
C12 = C_noise(1:col/2,row/2+1:end);
C21 = C_noise(col/2+1:end,1:row/2);
C22 = C_noise(col/2+1:end,row/2+1:end);
Qk = C22'*C12;



