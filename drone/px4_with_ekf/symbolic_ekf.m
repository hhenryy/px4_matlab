%% Extended Kalman Filter for Quadcopter

%% Symbols
Ts = 1/250;

% body angles, rates
syms P Q R

% body properties
syms m g Izz Iyy Ixx d r_D

% Euler 3-2-1 attitude parameters 
syms phi theta psi

% moments
syms L M N

% accelerations
syms U V W

% disturbances
syms Fx Fy Fz Mx My Mz

% thrust variables
syms del_e del_t del_r del_a del_er del_tr del_rr del_ar 

% motor constants
syms tau

% NED - coordinates
syms N E D

%% Inertial Equations
phi_dot   = 1*P + sin(phi)*tan(theta)*Q + cos(phi)*tan(theta)*R;
theta_dot = 0*P + cos(phi)*Q            + -sin(phi)*R;
psi_dot   = 0*P + sin(phi)*sec(theta)*Q + cos(phi)*sec(theta)*R;
%% Motor Equations
dot_del_t = (-del_t + del_tr)/tau;
dot_del_a = (-del_a + del_ar)/tau;
dot_del_e = (-del_e + del_er)/tau;
dot_del_r = (-del_r + del_rr)/tau;
%% Force Equations
U_dot = 1/m*( -m*g*sin(theta) + Fx) - W*Q + V*R;
V_dot = 1/m*( m*g*cos(theta)*sin(phi) + Fy) - U*R + W*P;
W_dot = 1/m*( m*g*cos(theta)*cos(phi) - del_t - d*del_a + Fz) - V*P + U*Q;
%% Moment Equations
P_dot = 1/Ixx * (d*del_a) - ( (Q*R)/Ixx )*(Izz - Iyy);
Q_dot = 1/Iyy * (d*del_e) - (P*R/Iyy)*(Ixx - Izz);
R_dot = 1/Izz * (r_D*del_r) - (P*Q/Izz)*(Izz - Ixx);
%% Relationship Equations


%% disturbance Jac
func = [U_dot,V_dot,W_dot,P_dot,Q_dot,R_dot,phi_dot,theta_dot,psi_dot];
vars =[U,V,W,P,Q,R,phi,theta,psi]
jac = jacobian(func,vars);

jac = eye(9) + jac*Ts


F = [[U_dot;V_dot;W_dot;P_dot;Q_dot;R_dot;phi_dot;theta_dot;psi_dot]]

