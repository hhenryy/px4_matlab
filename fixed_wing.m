clear all
%% Aerodynamic Constants
Cn_beta = 0.0860;
Cn_p = -0.0251;
Cn_r = -0.1250;
Cn_delta_A = -0.0065;
Cn_delta_R = -0.1129;

Cl_beta = -0.0331;
Cl_p = -0.4248;
Cl_r = 0.0450;
Cl_delta_A = -0.3731;
Cl_delta_R = 0.0080;

Cy_beta = -0.2777;
Cy_p = 0.0102;
Cy_r = 0.2122;
Cy_delta_A = -0.0077;
Cy_delta_R = 0.2303;

Cm_0 = 0.0;
Cm_alpha = -0.2954;
Cm_q = -10.281;
Cm_delta_E = -1.5852;

Cl_0 = 0.0;
Cl_alpha = 5.1309;
Cl_q = 7.7330;
Cl_delta_E = 0.7126;

Cd_0 = 0.02;
e = 0.85;


%% Geometry
c_bar = 0.3; % [m]
b = 1.73; % [m]
S = 0.5; % [m^2]
A = 5.97; % [1]

%% Inertia
m   = 5;     % [kg]
Ixx = 0.2;   % [kg*m^2]
Iyy = 0.360; % [kg*m^2]
Izz = 0.525; % [kg*m^2]
Ib=[Ixx,Iyy,Izz];

%% Trim point
V_trim = 30; %[m/s]
rho = 1.2754; % [kg/m^3]
g = 9.81; % [m/s^2]
q_trim = 0.5*rho*V_trim^2;
V_bar = 30; %[m/s]

trim_val = inv([Cl_alpha Cl_delta_E; Cm_alpha Cm_delta_E])*[(m*g)/(q_trim*S)-Cl_0;-Cm_0];

alpha_trim = trim_val(1);
delta_E_trim = trim_val(2);
Theta_trim = alpha_trim;



C_l_trim = Cl_0 + Cl_alpha*alpha_trim + Cl_delta_E*delta_E_trim;
C_d_trim = Cd_0 + C_l_trim^2/(pi*A*e);

C_x_trim = -C_d_trim*cos(alpha_trim) + C_l_trim*sin(alpha_trim);
C_z_trim = -C_l_trim*cos(alpha_trim) - C_d_trim*sin(alpha_trim);


T_trim =  q_trim*S*C_d_trim*cos(alpha_trim) - q_trim*S*C_l_trim*sin(alpha_trim)+m*g*sin(alpha_trim);

%% Propulsion
tau1 = 0.25; % [s]
T_max = 70;  % [N]
%%
trim_control = [0 0 delta_E_trim T_trim];
%sim('simulate_fixed_wing')


