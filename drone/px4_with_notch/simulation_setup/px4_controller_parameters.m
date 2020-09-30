%% PX4 Control Gains

%% Body Rates

% D-Term Low-pass Filter
MC_DTERM_CUTOFF = 0;

% Filter
[br_filt_num, br_filt_denom] = filter(MC_DTERM_CUTOFF, sim_freq);

% *************************************************************************
% Pitch
% *************************************************************************

% Pitch Rate Gains
MC_PITCHRATE_P = 0.06;
MC_PITCHRATE_I = 0.2;
MC_PITCHRATE_D = 0.0017;

% Pitch rate limit (deg/s)
MC_PITCHRATE_MAX = 220;

% Integrator limit
MC_PR_INT_LIM = 0.3;

% *************************************************************************
% Roll
% *************************************************************************

% Roll Rate Gains
MC_ROLLRATE_P = 0.06;
MC_ROLLRATE_I = 0.2;
MC_ROLLRATE_D = 0.0017;

% Roll rate limit (deg/s)
MC_ROLLRATE_MAX = 220;

% Integrator limit
MC_RR_INT_LIM = 0.3;

% *************************************************************************
% Yaw
% *************************************************************************

% Yaw Rate Gains
MC_YAWRATE_P = 0.15;
MC_YAWRATE_I =  0.2;
MC_YAWRATE_D = 0.0;

% Yaw rate limit (deg/s)
MC_YAWRATE_MAX = 200;

% Integrator limit
MC_YR_INT_LIM = 0.3;

%% Attitude

MPC_TILTMAX_AIR = 25; % Maximum tilt angle (deg)

% *************************************************************************
% Pitch
% *************************************************************************

% Pitch Angle Controller
MC_PITCH_P = 2.0;

% *************************************************************************
% Roll
% *************************************************************************

% Roll Angle Controller
MC_ROLL_P = 2.0;

% *************************************************************************
% Yaw
% ***********************************************************************

% Yaw Angle Controller
MC_YAW_P = 1;

%% Thrust (Velocity outputs inertial thrust which is converted to an attitude)

MPC_THR_MAX = 1.0;
MPC_THR_MIN = 0.02;

%% Velocity

% D-Term Low-pass Filter
MPC_VELD_LP = 5;

[vel_filt_num, vel_filt_denom] = filter(MPC_VELD_LP, sim_freq);

% Maximum velocities (m/s)
MPC_XY_VEL_MAX = 4; %change in PX4
MPC_Z_VEL_MAX_UP = 4; % Up (Rising)
MPC_Z_VEL_MAX_DN = 2; % Down (falling)

% *************************************************************************
% XY
% *************************************************************************

% Horizontal Velocity Controller
MPC_XY_VEL_P = 0.0644;
MPC_XY_VEL_I = 0.0194;
MPC_XY_VEL_D = 0.0068;

% *************************************************************************
% Z
% *************************************************************************

% Vertical Velocity Controller
MPC_Z_VEL_P = 0.2;
MPC_Z_VEL_I = 0.02;
MPC_Z_VEL_D = 0;

%% Position

% *************************************************************************
% XY
% *************************************************************************

% Horizontal Position Controller
MPC_XY_P = 0.25822;

% *************************************************************************
% Z
% *************************************************************************

% Vertical Position Controller
MPC_Z_P = 0.5;

%% Notch Filter 
s = tf('s');
wc = 5;
o = 2;
notch = (s^2 + wc^2)/(s^2 + o*s + wc^2);

%% Functions

% Discrete-Time Filter
function [num, denom] = filter(fc, fs)
    ohm = tan(pi*fc/fs);
    c = 1 + sqrt(2)*ohm + ohm^2;
    b0 = (ohm^2)/c;
    b1 = 2*b0;
    b2 = b0;
    a1 = 2*(ohm^2 - 1)/c;
    a2 = (1 - sqrt(2)*ohm + ohm^2)/c;
    num = [b0 b1 b2];
    denom = [1 a1 a2];
end