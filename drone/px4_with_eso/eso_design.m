%% PX4 Controller Design for X based rotary wing UAV
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


figure(1);
bode(G__Omega_PX4)
title('$G^{PX4}_{\Omega_{Y}}(s)$','FontSize',16,'Interpreter','latex' );
grid on

figure(2);
rlocus(G__Omega_PX4)
title('$G^{PX4}_{\Omega_{Y}}(s)$','FontSize',16,'Interpreter','latex' );

rltool(G__Omega_PX4)

D__Omega_Y = MC_PITCHRATE_P + 1/s*MC_PITCHRATE_I + s*MC_PITCHRATE_D;
%% Closed Loop Dynamics of Angular Rate Plant
G__Omega_PX4_CL = (D__Omega_Y*G__Omega_PX4)/(1+D__Omega_Y*G__Omega_PX4);
G__Omega_PX4_CL = factorise_tf(G__Omega_PX4_CL);

fprintf("Zeros of Closed Loop Angular Rate Plant: %f \n", zero(G__Omega_PX4_CL))
fprintf("==================================================================\n");
fprintf("Poles of Closed Loop Angular Rate Plant: %f \n", pole(G__Omega_PX4_CL))

figure(3)
bode(G__Omega_PX4_CL)
title('$G^{PX4}_{\Omega_{Y,cl}}(s)$','FontSize',16,'Interpreter','latex' );
grid on

figure(4)
rlocus(G__Omega_PX4_CL)
title('$G^{PX4}_{\Omega_{Y,cl}}(s)$','FontSize',16,'Interpreter','latex' );
grid on



%%
fprintf("Zeros of Closed Loop Angular Rate Plant: %f \n", zero(G__Omega_PX4_CL))
fprintf("==================================================================\n");
fprintf("Poles of Closed Loop Angular Rate Plant: %f \n", pole(G__Omega_PX4_CL))


%% Angle controller design
G__q2_PX4 = G__Omega_PX4_CL*(1/s);

%% Closed Loop Dynamics of Angle Plant
G__q2_PX4_CL = (MC_PITCH_P*G__q2_PX4)/(1+MC_PITCH_P*G__q2_PX4);
G__q2_PX4_CL = factorise_tf(G__q2_PX4_CL);



figure(5)
bode(G__Omega_PX4_CL)
hold on 
bode(G__q2_PX4_CL)
legend('angle cl','angle rate cl')
grid on




%% Linear Velocity Controller
G__V_N_PX4 = G__q2_PX4_CL*(4*Tmax/m)*(1/s);

D__V_N = MPC_XY_VEL_P + MPC_XY_VEL_I/s + s*MPC_XY_VEL_D;

%% Closed Loop Dynamics of Linear Velocity plant
G__V_N_PX4_CL = (D__V_N*G__V_N_PX4)/(1+D__V_N*G__V_N_PX4);
G__V_N_PX4_CL = factorise_tf(G__V_N_PX4_CL);


figure(3)
bode(G__V_N_PX4_CL)
title('$G^{PX4}_{V_{N},cl}(s)$','FontSize',16,'Interpreter','latex' );
grid on


%% Position Controller
G__X_N = G__V_N_PX4_CL*(1/s);

[b,a] = tfdata(G__X_N,'v');

pole(G__X_N)

A = diag(ones(length(a)-2,1),1);
A(end,:) = -a(2:end);
B = zeros(length(a)-1,1);


sigma = -0.1;

poles_eso = [sigma, 3*sigma, 3*sigma, 3*sigma, 3*sigma, 3*sigma, 3*sigma];



%% desired pole location

L = acker(A',C',poles_eso)';

% [cl_b,cl_a] = ss2tf(A-B*K,B,C,D);
% SYS_CLOSED = tf(cl_b,cl_a);

[comp_b,comp_a] = ss2tf(A-L*C,B,C,D);
SYS_COMP = tf(comp_b,comp_a);

figure(3)
pzmap(SYS_COMP)
hold on
grid on

