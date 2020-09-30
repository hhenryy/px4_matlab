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


%%

%% Convert to control canonical form
OBSERVER_SYS = canon(G__Omega_PX4,'companion');

A_CONTROL_SYS = OBSERVER_SYS.A';
B_CONTROL_SYS = OBSERVER_SYS.C';
C_CONTROL_SYS = OBSERVER_SYS.B';
D_CONTROL_SYS = OBSERVER_SYS.D;

%% Convert to state-space design:

NUM = cell2mat(G__Omega_PX4.NUM);
DEM = cell2mat(G__Omega_PX4.DEN);

A = [0 1 0; 0 0 1; 0 0 -500 ];
B = [0;B_CONTROL_SYS(end);-500*B_CONTROL_SYS(end)];
C = [1 0 0];
D = 0;
%%


eig(A)

eso_poles = [-20+20j -20-20j -500.00001];

K_poles = pole(G__Omega_PX4_CL)';

L = acker(A',C',eso_poles)'

[cl_b,cl_a] = ss2tf(A-B*K_poles,B,C,D);
SYS_CLOSED = tf(cl_b,cl_a);

[comp_b,comp_a] = ss2tf(A-L*C,B,C,D);
SYS_COMP = tf(comp_b,comp_a);

% pole(SYS_COMP)
wd = 30

L = [3*wd 3*wd^2 wd^3]'
% bode(SYS_COMP)
%%

G__Omega_PX4_OBSERVER_SYS = canon(G__Omega_PX4,'companion');
G__Omega_PX4_A_CONTROL_SYS = G__Omega_PX4_OBSERVER_SYS.A';
G__Omega_PX4_B_CONTROL_SYS = G__Omega_PX4_OBSERVER_SYS.C';
G__Omega_PX4_C_CONTROL_SYS = G__Omega_PX4_OBSERVER_SYS.B';
G__Omega_PX4_D_CONTROL_SYS = G__Omega_PX4_OBSERVER_SYS.D;

%% Angular Rate Extended State Observer
size = length(G__Omega_PX4_A_CONTROL_SYS);
A_angular_rate = diag(ones(size,1),1);
A_angular_rate(end,2:end) = G__Omega_PX4_A_CONTROL_SYS(end,1:end);
B_angular_rate = zeros(size+1,1);
B_angular_rate(1:size,1) = G__Omega_PX4_B_CONTROL_SYS;
B_angular_rate(end,1) = dot(G__Omega_PX4_B_CONTROL_SYS(2:end),G__Omega_PX4_A_CONTROL_SYS(end,2:end));
C_angular_rate_disturb = [0 0 1];
C_angular_rate = [1 0 0];

