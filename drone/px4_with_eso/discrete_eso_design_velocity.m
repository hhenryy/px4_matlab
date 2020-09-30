%% ESO Design with PX4 Controller with X based rotary wing UAV
% Design of controllers is based on using Anton Erasmus Thesis and notation
addpath(genpath('simulation_setup'));

clear model_parameters;
clear setup_simulation;
%% Constants
setup_simulation;
% model_parameters;
s = tf('s');

%% Angular Rate Plant Dynamics
G__Omega_PX4 = (2*Tmax)*(d/Iyy)*(1/tau_T)*(1/s)*(1/(s+1/tau_T));

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
G__V_N_PX4 = G__q2_PX4_CL*(4*Tmax/m)*(1/s);

D__V_N = MPC_XY_VEL_P + MPC_XY_VEL_I/s + s*MPC_XY_VEL_D;

%% Closed Loop Dynamics of Linear Velocity plant
G__V_N_PX4_CL = (D__V_N*G__V_N_PX4)/(1+D__V_N*G__V_N_PX4);
G__V_N_PX4_CL = factorise_tf(G__V_N_PX4_CL);

%%
fprintf("Zeros of Closed Loop Velocity Plant: %f \n", zero(G__V_N_PX4_CL))
fprintf("==================================================================\n");
fprintf("Poles of Closed Loop Velocity Plant: %f \n", pole(G__V_N_PX4_CL))


%% Convert to Control Canonical;
G__V_N_PX4_dSYS_OBSERVER_SYS = canon(G__V_N_PX4,'companion');
G__V_N_PX4_A_OBSERVER_SYS = G__V_N_PX4_dSYS_OBSERVER_SYS.A';
G__V_N_PX4_B_OBSERVER_SYS = G__V_N_PX4_dSYS_OBSERVER_SYS.C';
G__V_N_PX4_C_OBSERVER_SYS = G__V_N_PX4_dSYS_OBSERVER_SYS.B';
G__V_N_PX4_D_OBSERVER_SYS = G__V_N_PX4_dSYS_OBSERVER_SYS.D;


dG__V_N_PX4 = c2d(G__V_N_PX4,1/sim_freq,'zoh');

%%
size = length(G__V_N_PX4_A_OBSERVER_SYS);
G__V_N_PX4_A = diag(ones(size,1),1);
G__V_N_PX4_A(end,2:end) = G__V_N_PX4_A_OBSERVER_SYS(end,1:end);
G__V_N_PX4_B = zeros(size+1,1);
G__V_N_PX4_B(1:size,1) = G__V_N_PX4_B_OBSERVER_SYS;
G__V_N_PX4_C = zeros(1,size+1);
G__V_N_PX4_C(1,1:size) = G__V_N_PX4_C_OBSERVER_SYS;
G__V_N_PX4_B(end,1) = dot(G__V_N_PX4_B_OBSERVER_SYS(3:end),G__V_N_PX4_A_OBSERVER_SYS(5,3:end));
D =0;
%%
[G__V_N_PX4_cb,G__V_N_PX4_ca]  = ss2tf(G__V_N_PX4_A,G__V_N_PX4_B,G__V_N_PX4_C,D);
G__V_N_PX4_dSYS = c2d(tf(G__V_N_PX4_cb, G__V_N_PX4_ca),1/sim_freq,'zoh')

G__V_N_PX4_dSYS_OBSERVER_SYS = canon(G__V_N_PX4_dSYS,'companion');

G__V_N_PX4_A = G__V_N_PX4_dSYS_OBSERVER_SYS.A';
G__V_N_PX4_B = G__V_N_PX4_dSYS_OBSERVER_SYS.C';
G__V_N_PX4_C = G__V_N_PX4_dSYS_OBSERVER_SYS.B';
D = G__V_N_PX4_dSYS_OBSERVER_SYS.D;


%%


G__V_N_PX4_eso_poles = [-20 -20  -20 -120 -120 -120];
G__V_N_PX4_d_eso_poles = exp(G__V_N_PX4_eso_poles*0.004);
format long
G__V_N_PX4_L = acker(G__V_N_PX4_A',G__V_N_PX4_C',G__V_N_PX4_d_eso_poles)';
