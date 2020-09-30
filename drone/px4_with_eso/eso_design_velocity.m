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
OBSERVER_SYS = canon(G__V_N_PX4,'companion');
A_OBSERVER_SYS = OBSERVER_SYS.A';
B_OBSERVER_SYS = OBSERVER_SYS.C';
C_OBSERVER_SYS = OBSERVER_SYS.B';
D_OBSERVER_SYS = OBSERVER_SYS.D;

%%
size = length(A_OBSERVER_SYS);
A = diag(ones(size,1),1);
A(end,2:end) = A_OBSERVER_SYS(end,1:end);
B = zeros(size+1,1);
B(1:size,1) = B_OBSERVER_SYS;
C = zeros(1,size+1);
C(1,1:size) = C_OBSERVER_SYS;
B(end,1) = dot(B_OBSERVER_SYS(3:end),A_OBSERVER_SYS(5,3:end));


%%

naturals_poles = eig(A)


eso_poles = [-4.76357 naturals_poles(2) naturals_poles(3) naturals_poles(4) -4.76357 -4.76357]
format long
L = acker(A',C',eso_poles)'
