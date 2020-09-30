%% ESO Design with PX4 Controller with X based rotary wing UAV
% Design of controllers is based on using Anton Erasmus Thesis and notation
addpath(genpath('simulation_setup'));

clear model_parameters;
clear setup_simulation;
%% Constants
setup_simulation;
model_parameters;
s = tf('s');

%% Angular Rate Plant Dynamics
G__Omega_PX4 = (2*Tmax)*(d/Iyy)*(1/tau_T)*(1/s)*(1/(s+1/tau_T));

D__Omega_Y = MC_PITCHRATE_P + 1/s*MC_PITCHRATE_I + s*MC_PITCHRATE_D;
%% Closed Loop Dynamics of Angular Rate Plant
G__Omega_PX4_CL = (D__Omega_Y*G__Omega_PX4)/(1+D__Omega_Y*G__Omega_PX4);
G__Omega_PX4_CL = factorise_tf(G__Omega_PX4_CL);


%%
fprintf("Zeros of Closed Loop Angular Rate Plant: %f \n", zero(G__Omega_PX4_CL))
fprintf("==================================================================\n");
fprintf("Poles of Closed Loop Angular Rate Plant: %f \n", pole(G__Omega_PX4_CL))

NUM = cell2mat(G__Omega_PX4.NUM);
DEM = cell2mat(G__Omega_PX4.DEN);

tf2ss(NUM,DEM)

%% Convert to state-space design:

d_G__Omega_PX4 = c2d(G__Omega_PX4,1/sim_freq,'zoh');


% [cb,ca]  = ss2tf(A,B,C,D)
% dSYS = c2d(G__Omega_PX4,1/sim_freq,'zoh')
G__Omega_PX4_OBSERVER_SYS = canon(G__Omega_PX4,'companion');

A_CONTROL_SYS = G__Omega_PX4_OBSERVER_SYS.A'
G__Omega_PX4_B_CONTROL_SYS = G__Omega_PX4_OBSERVER_SYS.C'
G__Omega_PX4_C_CONTROL_SYS = G__Omega_PX4_OBSERVER_SYS.B'
G__Omega_PX4_D_CONTROL_SYS = G__Omega_PX4_OBSERVER_SYS.D


G__Omega_PX4_A = [0 1 0; 0 0 1; 0 0 -500 ];
G__Omega_PX4_B = [0;G__Omega_PX4_B_CONTROL_SYS(end);-500*G__Omega_PX4_B_CONTROL_SYS(end)];
G__Omega_PX4_C = [1 0 0];
G__Omega_PX4_D = 0;
[G__Omega_PX4_cb,G__Omega_PX4_ca]  = ss2tf(G__Omega_PX4_A,G__Omega_PX4_B,G__Omega_PX4_C,G__Omega_PX4_D);

%% Convert to state-space design:

d_G__Omega_PX4 = c2d(G__Omega_PX4,1/sim_freq,'zoh');


% [cb,ca]  = ss2tf(A,B,C,D)
% dSYS = c2d(G__Omega_PX4,1/sim_freq,'zoh')
G__Omega_PX4_OBSERVER_SYS = canon(G__Omega_PX4,'companion');

A_CONTROL_SYS = G__Omega_PX4_OBSERVER_SYS.A'
G__Omega_PX4_B_CONTROL_SYS = G__Omega_PX4_OBSERVER_SYS.C'
G__Omega_PX4_C_CONTROL_SYS = G__Omega_PX4_OBSERVER_SYS.B'
G__Omega_PX4_D_CONTROL_SYS = G__Omega_PX4_OBSERVER_SYS.D
G__Omega_PX4_dSYS = c2d(tf(G__Omega_PX4_cb, G__Omega_PX4_ca),1/sim_freq,'zoh')

G__Omega_PX4_OBSERVER_SYS = canon(G__Omega_PX4_dSYS,'companion');

G__Omega_PX4_A = G__Omega_PX4_OBSERVER_SYS.A'
G__Omega_PX4_B = G__Omega_PX4_OBSERVER_SYS.C'
G__Omega_PX4_C = G__Omega_PX4_OBSERVER_SYS.B'
G__Omega_PX4_D = G__Omega_PX4_OBSERVER_SYS.D
%% Angular Rate Extended State Observer

G__Omega_wn = 200;
G__Omega_zeta = 1;
G__Omega_pole1 = -G__Omega_wn*G__Omega_zeta + G__Omega_wn*sqrt(1-G__Omega_zeta^2)*1i;
G__Omega_pole2 = -G__Omega_wn*G__Omega_zeta - G__Omega_wn*sqrt(1-G__Omega_zeta^2)*1i;
G__Omega_p = [exp(G__Omega_pole1*0.004), exp(G__Omega_pole2*0.004),exp(-3*G__Omega_wn*0.004)];

G__Omega_L = acker(G__Omega_PX4_A',G__Omega_PX4_C',G__Omega_p)';
% L = [3*wd 3*wd^2 wd^3]';
%L = exp(L_angular_rate*0.004)





% A = [0 1 0; 0 0 1; 0 0 -500 ];
% B = [0;B_CONTROL_SYS(end);-500*B_CONTROL_SYS(end)];
% C = [1 0 0];
% D = 0;