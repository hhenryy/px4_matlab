%% Simulation
% Duration of simulation
sim_time = 10;
% Step inputs
pos_e_step = [0,2,0];
yaw_step = 0.00;

%% Setup
% Add subfolders to path
addpath(genpath('simulation_setup'));
setup_simulation;

%% Sensor Noise
sensor_noise = 0;

%% Disturbance
disturbance_on_off = 1;
force_disturbance = 0;
moment_disturbance = 1;

% Activate disturbance rejection
ADRC = 0;

%% Neural Network Effective Time Delay
ROS2_delay=1/sim_freq;


%% Run simulation
sim('px4_mc_sim');