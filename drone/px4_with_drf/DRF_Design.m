%% ESO Design with PX4 Controller with X based rotary wing UAV
% Design of controllers is based on using Anton Erasmus Thesis and notation
addpath(genpath('simulation_setup'));

%% Constants
setup_simulation;
model_parameters;
s = tf('s');

%% Angular Rate Plant Dynamics
G__Omega_PX4 = (2*max_T)*(d/Iyy)*(1/tau_T)*(1/s)*(1/(s+1/tau_T));

D__Omega_Y = MC_PITCHRATE_P + 1/s*MC_PITCHRATE_I + s*MC_PITCHRATE_D;
%% Closed Loop Dynamics of Angular Rate Plant
G__Omega_PX4_CL = (D__Omega_Y*G__Omega_PX4)/(1+D__Omega_Y*G__Omega_PX4);
G__Omega_PX4_CL = factorise_tf(G__Omega_PX4_CL);

%% Angle controller design
G__q2_PX4 = G__Omega_PX4_CL*(1/s);

%% Closed Loop Dynamics of Angle Plant
G__q2_PX4_CL = (MC_PITCH_P*G__q2_PX4)/(1+MC_PITCH_P*G__q2_PX4);
G__q2_PX4_CL = factorise_tf(G__q2_PX4_CL);

%% Linear Velocity Controller
G__V_N_PX4 = G__q2_PX4_CL*(4*max_T/m)*(1/s);

D__V_N = MPC_XY_VEL_P + MPC_XY_VEL_I/s + s*MPC_XY_VEL_D;

%% Closed Loop Dynamics of Linear Velocity plant
G__V_N_PX4_CL = (D__V_N*G__V_N_PX4)/(1+D__V_N*G__V_N_PX4);
G__V_N_PX4_CL = factorise_tf(G__V_N_PX4_CL);

%%

pzmap(G__V_N_PX4)
pole(G__V_N_PX4)
zero(G__V_N_PX4)

%%
pzmap(G__Omega_PX4)