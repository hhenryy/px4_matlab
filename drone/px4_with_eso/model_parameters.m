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
g = 9.80665; % gravitational constant
tau_T = 0.002; % Motor time constant
virtual_controls_mat = [1 1 1 1; -1/sqrt(2) 1/sqrt(2) 1/sqrt(2) -1/sqrt(2); 1/sqrt(2) -1/sqrt(2) 1/sqrt(2) -1/sqrt(2); 1 1 -1 -1]; % Mixin matrix for X Quadrotormixin_mat = virtual_controls_mat'; % mixin matrix
mixin_matrix = virtual_controls_mat'; % mixin matrix
max_total_T_kg = 27.4/9.81; % max thrust of total quadrotor in kg (total), Not just per motor


% Propulsion:
max_total_T = max_total_T_kg * g; % maximum total thrust
max_T = max_total_T / 4; % maximum thrust per motor
Tmax = max_T;

hover_perc = m*g / max_total_T; % hover percentage of full throttle
hover_total_T = max_total_T * hover_perc; % total hover thrust
hover_T = hover_total_T / 4; % hover thrust per motor



% Aerodynamic:
C_D = [0.064; 0.067; 0.089].*0; % From Pierro - need to calculate in flight test
wind = [0; 0; 0]; % Wind constants





