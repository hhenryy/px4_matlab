%% Simulation
% Duration of simulation
sim_time = 30;
% Step inputs
pos_e_step = [0,2,0];
yaw_step = 0.00;

%% Setup
% Add subfolders to path
addpath(genpath('simulation_setup'));
setup_simulation;

%% Sensor Noise
sensor_noise = 0;

%% DISTURBANCE SELECTOR
% Step
step_mag = 0.5; % second
step_delay = 1; % second

% Pulse 
pulse_amplitude = 0.1;
pulse_period = 40; % seconds

% Sinusoid
sinus_amplitude = 0.1;
sinus_bias = 0;
sinus_freq = 1;

% Ramp 
ramp_slope = 0.05;
ramp_max = 0.5;
ramp_start = 1;


%% Run simulation
sim('px4_mc_sim_distrb_ekf');