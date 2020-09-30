%% s-parameter
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

%% Position Plant
G__X_N = G__V_N_PX4_CL*(1/s);

%% Position Contoller
P__X_N = MPC_XY_P;

%% Closed Loop Dynamics of Position Plant
G__X_N_PX4_CL = G__X_N*P__X_N/(1+G__X_N*P__X_N);
G__X_N_PX4_CL = factorise_tf(G__X_N_PX4_CL);

%% Position Q-Filter Design
simplified_velocity_plant = 1/s;
simplified_velocity_plant_CL = D__V_N*simplified_velocity_plant/(1+D__V_N*simplified_velocity_plant);
simplified_velocity_plant_CL = factorise_tf(simplified_velocity_plant_CL);

simplified_position_plant = simplified_velocity_plant_CL*(1/s);


NUM = cell2mat(simplified_position_plant.NUM);
DEN = cell2mat(simplified_position_plant.DEN);
[R_pos_plant,P_pos_plant,K_pos_plant]=residue(DEN,NUM);

Q_zeta= 10;
Q_wn = 1;

Q_2nd_NUM = [0 0 1];
Q_2nd_DEM = [1 2*Q_zeta*Q_wn Q_wn^2];




