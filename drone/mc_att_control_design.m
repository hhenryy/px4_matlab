%% PX4 Controller Design for X based rotary wing UAV
% Design of controllers is based on using Anton Erasmus Thesis and notation

clear model_parameters;
%% Constants
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

%% Rate controller Design
rltool(G__Omega_PX4)
%% Determinining PID Gains
% D(s) = a(s+b)(s+c)/(s)

a = 0.0076631;
b = 1.405;
c = 560.4;

B = [a; a*(b+c); a*b*c];
A = [0 0 1; 1 0 0; 0 1 0];
PID_gains = A\B;
disp(PID_gains)

P = PID_gains(1);
I = PID_gains(2);
D = PID_gains(3);

D__Omega_Y = P + 1/s*I + s*D;
%% Closed Loop Dynamics of Angular Rate Plant
G__Omega_PX4_CL = (D__Omega_Y*G__Omega_PX4)/(1+D__Omega_Y*G__Omega_PX4);

figure(3)
bode(G__Omega_PX4_CL)
title('$G^{PX4}_{\Omega_{Y,cl}}(s)$','FontSize',16,'Interpreter','latex' );
grid on

figure(4)
rlocus(G__Omega_PX4_CL)
title('$G^{PX4}_{\Omega_{Y,cl}}(s)$','FontSize',16,'Interpreter','latex' );
grid on

%% Angle controller design
G__q2_PX4 = G__Omega_PX4_CL*(1/s);
rltool(G__q2_PX4)
%% Determinining P Gains for Angle Rate Controller
P_q2 = 45;
%% Closed Loop Dynamics of Angle Plant
G__q2_PX4_CL = (P_q2*G__q2_PX4)/(1+P_q2*G__q2_PX4);

figure(5)
bode(G__Omega_PX4_CL)
hold on 
bode(G__q2_PX4_CL)
legend('angle cl','angle rate cl')
grid on

%% Linear Velocity Controller
G__V_N_PX4 = G__q2_PX4_CL*(4*Tmax/m)/(s);

%% Root locus design of Linear velocity Controller
rltool(G__V_N_PX4);
%% Determinining PID Gains
% D(s) = a(s+b)(s+c)/(s)

a = 0.0076631;
b = 1.405;
c = 560.4;

B = [a; a*(b+c); a*b*c];
A = [0 0 1; 1 0 0; 0 1 0];
PID_gains = A\B;
disp(PID_gains)

P = PID_gains(1);
I = PID_gains(2);
D = PID_gains(3);

D__V_N = P + 1/s*I + s*D;

%% Closed Loop Dynamics of Linear Velocity plant
G__V_N_PX4_CL = (D__V_N*G__V_N_PX4)/(1+D__V_N*G__V_N_PX4);

figure(3)
bode(G__V_N_PX4_CL)
title('$G^{PX4}_{V_{N},cl}(s)$','FontSize',16,'Interpreter','latex' );
grid on

figure(4)
rlocus(G__V_N_PX4_CL)
title('$G^{PX4}_{V_{N},cl}(s)$','FontSize',16,'Interpreter','latex' );

%% Position Controller
G__X_N = G__V_N_PX4_CL*(1/s);
%% Design of Position Controller
rltool(G__X_N);


