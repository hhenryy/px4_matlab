%% Longitudinal Controllers for Fixed Wing UAV
clear all;
%% Constants
constants;
%% A Matrix
A11 = -1/tau_T;
A12 = 0;
A13 = 0;
A14 = 0;

A21 = d/Iyy;
A22 = 0;
A23 = 0;
A24 = 0;

A31 = 0;
A32 = 1;
A33 = 0;
A34 = 0;

A41 = 0;
A42 = 0;
A43 = -g;
A44 = 0;

A_long = [A11 A12 A13 A14; A21 A22 A23 A24; A31 A32 A33 A34; A41 A42 A43 A44]

poles_A_long = eig(A_long);

figure(1)
plot(poles_A_long,'x','LineWidth',2)
title("Pole Locations of Longitudinal Model",'Interpreter','latex')
grid on

%% B Matrix
B11 = 0;
B12 = 1/m;

B21 = -(q_trim*S)/(m*V_trim)*Cl_delta_E;
B22 = 0;

B31 = (q_trim*S*c_bar/Iyy)*Cm_delta_E;
B32 = 0;

B41 = 0;
B42 = 0;

B_long = [B11 B12; B21 B22; B31 B32; B41 B42];
C_q = [0 0 1 0];
C_alpha = [0 1 0 0 ];
C_velocity = [ 1 0 0 0];
C_theta = [0 0 0 1];

s = tf('s');

%% Pitch Rate Damper
[n,d] = ss2tf(A_long,B_long(:,1),C_q,0);
pitch_elevator_tf = tf(n,d);

[n,d] = ss2tf(A_long,B_long(:,1),C_alpha,0);
alpha_elevator_tf=tf(n,d);

[n,d] = ss2tf(A_long,B_long(:,1),C_theta,0);
theta_elevator_tf = tf(n,d);

[n,d] = ss2tf(A_long,B_long(:,1),C_velocity,0);
velocity_elevator_tf = tf(n,d);


poles_pitch_elevator_tf = pole(pitch_elevator_tf);

plot(poles_pitch_elevator_tf,'x','LineWidth',2)


figure(2);
bode(pitch_elevator_tf);
grid on
title('$$\frac{q(s)}{\delta_{E}(s)}$$','Interpreter','latex' );

figure(3);
bode(alpha_elevator_tf);
grid on
title('$$\frac{\alpha(s)}{\delta_{E}(s)}$$','Interpreter','latex' );

figure(4);
bode(velocity_elevator_tf);
grid on
title('$$\frac{\bar{v}(s)}{\delta_{E}(s)}$$','Interpreter','latex' );

figure(5);
bode(theta_elevator_tf);
grid on
title('$$\frac{\theta(s)}{\delta_{E}(s)}$$','Interpreter','latex' );




%rltool(pitch_elevator_tf);
Kq = -0.030851;
A_PRD = (A_long - B_long*[Kq;0]*C_q);

poles_A_PRD = eig(A_PRD);

[n,d] = ss2tf(A_PRD,B_long(:,1),C_q,0);
pitch_elevator_ptch_dmpr_tf = tf(n,d);

[n,d] = ss2tf(A_PRD,B_long(:,1),C_alpha,0);
alpha_elevator_ptch_dmpr_tf = tf(n,d);

[n,d] = ss2tf(A_PRD,B_long(:,1),C_theta,0);
theta_elevator_ptch_dmpr_tf = tf(n,d);

[n,d] = ss2tf(A_PRD,B_long(:,1),C_velocity,0);
velocity_elevator_ptch_dmpr_tf = tf(n,d);

%%
figure(11);
bode(pitch_elevator_ptch_dmpr_tf)
hold on
bode(pitch_elevator_tf);
grid on
title('$$\frac{q(s)}{\delta_{E}(s)}$$','Interpreter','latex' );
legend('with pitch rate damper','with no pitch rate damper')

figure(12);
bode(alpha_elevator_ptch_dmpr_tf)
hold on
bode(alpha_elevator_tf)
grid on
title('$$\frac{\alpha(s)}{\delta_{E}(s)}$$','Interpreter','latex' );
legend('with pitch rate damper','with no pitch rate damper')

figure(13);
bode(theta_elevator_ptch_dmpr_tf)
hold on
bode(theta_elevator_tf)
grid on
title('$$\frac{\theta(s)}{\delta_{E}(s)}$$','Interpreter','latex' );
legend('with pitch rate damper','with no pitch rate damper')

figure(14);
bode(velocity_elevator_ptch_dmpr_tf)
hold on
bode(velocity_elevator_tf);
grid on
title('$$\frac{\bar{v}(s)}{\delta_{E}(s)}$$','Interpreter','latex' );
legend('with pitch rate damper','with no pitch rate damper')

figure(16);
plot(poles_A_long,'x','LineWidth',2);
hold on
plot(poles_A_PRD,'x','LineWidth',2);
plot(poles_pitch_elevator_tf,'x','LineWidth',2)
legend('Openloop poles of Longitudinal Dynamics','CL poles of Longitudinal with pitch rate damper','Pole location of pitch elevator tf')
grid on
%% Airspeed and Climb Rate Controller
C_vel_climbrate = [1 0 0 0; 0 -V_trim 0 V_trim];

A_ASCR_OL =  [A_PRD, zeros(4,2); C_vel_climbrate, zeros(2,2) ];
B_ASCR_OL = [B_long;zeros(2,2)];

poles_ASCR_OL = eig(A_ASCR_OL);
figure(4)
plot(poles_ASCR_OL,'o','LineWidth',2);
title('Pole location of ASCR openloop plant','Interpreter','latex');
grid on

max_v_trim = 35^2;
max_alpha = (10*pi/180)^2;
max_q = (30*pi/180)^2;
max_theta = (10*pi/180)^2;
max_v_err = (10)^2;
max_clmb_rt_err = 5^2;

max_eleron = 10;
max_thrust = 70;


Q = diag([1/max_v_trim, 1/max_alpha, 1/max_q, 1/max_theta, 1/max_v_err, 1/max_clmb_rt_err]);
R = diag([1/max_eleron, 1/max_thrust]);

K_ASCR = lqr(A_ASCR_OL,B_ASCR_OL,Q,R);

A_ASCR_CL = A_ASCR_OL - B_ASCR_OL*K_ASCR;


poles_ASCR_CL = eig(A_ASCR_CL);

figure(5)
plot(poles_ASCR_CL,'o','LineWidth',2)
title('Pole Location of ASCR Closed Loop')
grid on

figure(6)
plot(poles_ASCR_CL,'o','LineWidth',2)
hold on
plot(poles_ASCR_OL,'o','LineWidth',2);
legend('closed loop poles', 'openloop')
grid on



%% Altitude Controller

C_hdot = [0 -V_trim 0 V_trim zeros(1,2)];
B_CR = transpose([zeros(1,5) -1]);

A_altitude = [A_ASCR_CL zeros(6,1); C_hdot 0 ];

poles_altitude_OL = eig(A_altitude);

figure(7);
plot(poles_altitude_OL, 'o', 'LineWidth',2)
grid on

altitude_CL = 1/s*C_hdot * inv(s*eye(6)-A_ASCR_CL) * B_CR;

%rltool(altitude_CL)
