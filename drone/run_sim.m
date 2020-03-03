%% Simulation
% Duration of simulation
sim_time = 10;
% Step inputs
pos_e_step = [10,2,3];
yaw_step = 0.00;

%% Setup
% Add subfolders to path
addpath(genpath('simulation_setup'));
setup_simulation;

%% Sensor Noise
sensor_noise = 0;

%% Run simulation
sim('px4_mc_sim');