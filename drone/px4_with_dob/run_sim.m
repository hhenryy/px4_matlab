%% Simulation
% Duration of simulation
sim_time = 500;
% Step inputs
pos_e_step = [0,0,0];
yaw_step = 0.00;

%% Setup
% Add subfolders to path
addpath(genpath('simulation_setup'));
setup_simulation;

%% Sensor Noise
sensor_noise = 0;

%% DISTURBANCE SELECTOR
% Step
step_mag = -0.1; % second
step_delay = 3; % second

% Pulse 
pulse_amplitude = 0.1;
pulse_period = 40; % seconds

% Sinusoid
sinus_amplitude = 0.2;
sinus_bias = 0;
sinus_freq = 10;

% Ramp 
ramp_slope = 0.05;
ramp_max = 0.5;
ramp_start = 1;

%% Run simulation
sim('px4_mc_dob_sim');