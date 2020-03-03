clear all;

%% Constants
constants;


%% A Matrix

A11 = q_trim*S*Cy_beta/(m*V_trim);
A12 = q_trim*S*b/(m*V_trim*2*V_trim)*Cy_p;
A13 = q_trim*S/(m*V_trim)*Cy_r - 1;
A14 = g*cos(Theta_trim)/V_trim;

A21 = q_trim*S*b/Ixx*Cl_beta;
A22 = q_trim*S*b/(Ixx*2*V_trim)*Cl_p;
A23 = q_trim*S*b*b/(Ixx*2*V_trim)*Cl_r;
A24 = 0;

A31 = q_trim*S*b/Ixx*Cn_beta;
A32 = q_trim*S*b*b/(Izz*2*V_trim)*Cl_p;
A33 = q_trim*S*b*b/(Izz*2*V_trim)*Cn_r;
A34 = 0;

A41 = 0;
A42 = 1;
A43 = tan(Theta_trim);
A44 = 0;


B11 = q_trim*S/(m*V_trim)*Cy_delta_A;
B12 = q_trim*S/(m*V_trim)*Cy_delta_R;

B21 = q_trim*S*b/(Ixx)*Cl_delta_A;
B22 = q_trim*S*b/(Ixx)*Cl_delta_R;

B31 = q_trim*S*b/Ixx*Cn_delta_A;
B32 = q_trim*S*b/Izz*Cn_delta_R;

B41 = 0;
B42 = 0;

A_lat = [A11 A12 A13 A14; A21 A22 A23 A24; A31 A32 A33 A34; A41 A42 A43 A44];
B_lat = [B11 B12; B21 B22; B31 B32; B41 B42];


%% Poles of Lateral

poles_lat = eig(A_lat);

figure(1)
plot(poles_lat,'x','LineWidth',2)
title("Pole Locations of Lateral Model",'Interpreter','latex')
grid on



%% Dutch Roll Damper
s = tf('s');
C_yaw = [0 0 1 0 ];
tau_w = 0.15;
K = -1;
Aw = -1/tau_w;
Bw = K;
Cw = -1/tau_w;
Dw = K;

washout_filter = tau_w*s/(tau_w*s+1);

%bode(washout_filter)

[n,d] = ss2tf(A_lat,B_lat(:,2),C_yaw,0);
yaw_rudder_tf = tf(n,d);


yaw_rudder_washout = yaw_rudder_tf/(1-yaw_rudder_tf*washout_filter);

rlocus(yaw_rudder_washout)
rltool(yaw_rudder_washout);


%% Roll angle controller

A_DRD = [A_lat - B_lat(:,2)*Dw*C_yaw -B_lat(:,2)*Cw; Bw*C_yaw Aw];
C_roll = [0 0 0 1 0];

[n,d] = ss2tf(A_DRD,[B_lat(:,1);0],C_roll,0);
roll_aileron_tf = tf(n,d);


figure(2);
bode(roll_aileron_tf)
rlocus(roll_aileron_tf)
rltool(roll_aileron_tf)













