%% Performance Analysis
% Determine the performance increase of using the ESO
% Assumption: We are at steady state and a disturbance occur at 1s

%% Simulation
% Duration of simulation
sim_time = 25;
% Step inputs
pos_e_step = [0,0,0];
yaw_step = 0;


activate_disturbance = 1;

% Step
step_mag = 0.1; % second
step_delay = 3; % second

% Sinusoid
sinus_amplitude = 0.2;
sinus_bias = 0;
sinus_freq = 3;

% Ramp 
ramp_slope = 0.05;
ramp_max = 0.15;
ramp_start = step_delay;

%% Setup
% Add subfolders to path
addpath(genpath('simulation_setup'));
setup_simulation;
ROS2_delay=5/sim_freq;    %% Communcication of 100 Hz

%% Vanilla Simulation
activate_disturbance = 1;
disturbance_rejection = 0;

%% Vanilla simulation, moment step disturbance
disp("Vanilla simulation, moment step disturbance");
option1 = 1;    % step disturbance    
option2 = 2;    % moment
simulation = sim('px4_mc_nn_comparisons_sim');
% moment step disturbance
m_step_disturbance_vanilla_pitch_rate_response=simulation.get('pitchrate').Data;
m_step_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;

%% Vanilla simulation, moment ramp disturbance
disp("Vanilla simulation, moment ramp disturbance");
option1 = 2;    % ramp disturbance    
option2 = 2;    % moment
simulation = sim('px4_mc_nn_comparisons_sim');
% moment ramp disturbance
m_ramp_disturbance_vanilla_pitch_rate_response=simulation.get('pitchrate').Data;
m_ramp_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;

%% Vanilla simulation, sinus moment disturbance
disp("Vanilla simulation, moment sinus disturbance");
option1 = 3;    % sinus disturbance    
option2 = 2;    % moment
simulation = sim('px4_mc_nn_comparisons_sim');
% moment ramp disturbance
m_sinus_disturbance_vanilla_pitch_rate_response=simulation.get('pitchrate').Data;
m_sinus_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;
%% nn Simulation
disturbance_rejection = 1;
activate_disturbance = 1;

%% nn simulation, moment step disturbance
disp("nn simulation, moment step disturbance");
option1 = 1;    % step disturbance    
option2 = 2;    % moment
simulation = sim('px4_mc_nn_comparisons_sim');
m_step_disturbance_nn_pitch_rate_response=simulation.get('pitchrate').Data;
m_step_disturbance_nn_Xvelocity_response=simulation.get('Xvelocity').Data;

%% nn simulation, moment ramp disturbance
disp("nn simulation, moment ramp disturbance");
option1 = 2;    % ramp disturbance    
option2 = 2;    % moment
simulation = sim('px4_mc_nn_comparisons_sim');
m_ramp_disturbance_nn_pitch_rate_response=simulation.get('pitchrate').Data;
m_ramp_disturbance_nn_Xvelocity_response=simulation.get('Xvelocity').Data;

%% nn simulation, moment sinus disturbance
disp("nn simulation, moment sinus disturbance");
option1 = 3;    % sinus disturbance    
option2 = 2;    % moment
simulation = sim('px4_mc_nn_comparisons_sim');
m_sinus_disturbance_nn_pitch_rate_response=simulation.get('pitchrate').Data;
m_sinus_disturbance_nn_Xvelocity_response=simulation.get('Xvelocity').Data;


time = simulation.get('pitchrate').Time;

%% FORCE SIMULATION VANILLA
disturbance_rejection = 0;
activate_disturbance = 1;
%% Vanilla simulation, moment step disturbance
disp("Vanilla simulation, force step disturbance");
option1 = 1;    % step disturbance    
option2 = 1;    % force
simulation = sim('px4_mc_nn_comparisons_sim');
% moment step disturbance
f_step_disturbance_vanilla_pitch_rate_response=simulation.get('pitchrate').Data;
f_step_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;

%% Vanilla simulation, force ramp disturbance
disp("Vanilla simulation, force ramp disturbance");
option1 = 2;    % ramp disturbance    
option2 = 1;    % force
simulation = sim('px4_mc_nn_comparisons_sim');
% moment ramp disturbance
f_ramp_disturbance_vanilla_pitch_rate_response=simulation.get('pitchrate').Data;
f_ramp_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;

%% Vanilla simulation, force force disturbance
disp("Vanilla simulation, force sinus disturbance");
option1 = 3;    % sinus disturbance    
option2 = 1;    % force
simulation = sim('px4_mc_nn_comparisons_sim');
% moment ramp disturbance
f_sinus_disturbance_vanilla_pitch_rate_response=simulation.get('pitchrate').Data;
f_sinus_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;

%% Neural networks FORCE SIMULATION
disturbance_rejection = 1;
activate_disturbance = 1;
%% nn simulation, force step disturbance
disp("nn simulation, force step disturbance");
option1 = 1;    % step disturbance    
option2 = 1;    % force
simulation = sim('px4_mc_nn_comparisons_sim');
f_step_disturbance_nn_pitch_rate_response=simulation.get('pitchrate').Data;
f_step_disturbance_nn_Xvelocity_response=simulation.get('Xvelocity').Data;
nn_force_step_estimation =simulation.get('force_disturbance_est').Data;

%% nn simulation, force ramp disturbance
disp("nn simulation, force ramp disturbance");
option1 = 2;    % ramp disturbance    
option2 = 1;    % force
simulation = sim('px4_mc_nn_comparisons_sim');
f_ramp_disturbance_nn_pitch_rate_response=simulation.get('pitchrate').Data;
f_ramp_disturbance_nn_Xvelocity_response=simulation.get('Xvelocity').Data;
nn_force_ramp_estimation =simulation.get('force_disturbance_est').Data;

%% nn simulation, force sinus disturbance
disp("nn simulation, force sinus disturbance");
option1 = 3;    % sinus disturbance    
option2 = 1;    % force
simulation = sim('px4_mc_nn_comparisons_sim');
f_sinus_disturbance_nn_pitch_rate_response=simulation.get('pitchrate').Data;
f_sinus_disturbance_nn_Xvelocity_response=simulation.get('Xvelocity').Data;
nn_force_sinus_estimation =simulation.get('force_disturbance_est').Data;


time = simulation.get('pitchrate').Time;

%% Moment Analysis for vanilla response

% ISE: Integrated Squared Error
% IAE: Integrated Absolute Error
% ITAE: Integrated Time Absolute Error

ISE_m_step_disturbance_vanilla_pitch_rate_response = trapz(1/sim_freq,m_step_disturbance_vanilla_pitch_rate_response.^2);
ISE_m_ramp_disturbance_vanilla_pitch_rate_response = trapz(1/sim_freq,m_ramp_disturbance_vanilla_pitch_rate_response.^2);
ISE_m_sinus_disturbance_vanilla_pitch_rate_response = trapz(1/sim_freq,m_sinus_disturbance_vanilla_pitch_rate_response.^2);


IAE_m_step_disturbance_vanilla_pitch_rate_response = trapz(1/sim_freq,abs(m_step_disturbance_vanilla_pitch_rate_response));
IAE_m_ramp_disturbance_vanilla_pitch_rate_response = trapz(1/sim_freq,abs(m_ramp_disturbance_vanilla_pitch_rate_response));
IAE_m_sinus_disturbance_vanilla_pitch_rate_response = trapz(1/sim_freq,abs(m_sinus_disturbance_vanilla_pitch_rate_response));


ITAE_m_step_disturbance_vanilla_pitch_rate_response = trapz(1/sim_freq,time.*abs(m_step_disturbance_vanilla_pitch_rate_response));
ITAE_m_ramp_disturbance_vanilla_pitch_rate_response = trapz(1/sim_freq,time.*abs(m_ramp_disturbance_vanilla_pitch_rate_response));
ITAE_m_sinus_disturbance_vanilla_pitch_rate_response = trapz(1/sim_freq,time.*abs(m_sinus_disturbance_vanilla_pitch_rate_response));



%% moment Analysis for nn Response

ISE_m_step_disturbance_nn_pitch_rate_response = trapz(1/sim_freq,m_step_disturbance_nn_pitch_rate_response.^2);
ISE_m_ramp_disturbance_nn_pitch_rate_response = trapz(1/sim_freq,m_ramp_disturbance_nn_pitch_rate_response.^2);
ISE_m_sinus_disturbance_nn_pitch_rate_response = trapz(1/sim_freq,m_sinus_disturbance_nn_pitch_rate_response.^2);


IAE_m_step_disturbance_nn_pitch_rate_response = trapz(1/sim_freq,abs(m_step_disturbance_nn_pitch_rate_response));
IAE_m_ramp_disturbance_nn_pitch_rate_response = trapz(1/sim_freq,abs(m_ramp_disturbance_nn_pitch_rate_response));
IAE_m_sinus_disturbance_nn_pitch_rate_response = trapz(1/sim_freq,abs(m_sinus_disturbance_nn_pitch_rate_response));


ITAE_step_disturbance_nn_pitch_rate_response = trapz(1/sim_freq,time.*abs(m_step_disturbance_nn_pitch_rate_response));
ITAE_ramp_disturbance_nn_pitch_rate_response = trapz(1/sim_freq,time.*abs(m_ramp_disturbance_nn_pitch_rate_response));
ITAE_sinus_disturbance_nn_pitch_rate_response = trapz(1/sim_freq,time.*abs(m_sinus_disturbance_nn_pitch_rate_response));

%% %% force Analysis for vanilla response

% ISE: Integrated Squared Error
% IAE: Integrated Absolute Error
% ITAE: Integrated Time Absolute Error

ISE_f_step_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,f_sinus_disturbance_vanilla_Xvelocity_response.^2);
ISE_f_ramp_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,f_ramp_disturbance_vanilla_Xvelocity_response.^2);
ISE_f_sinus_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,f_sinus_disturbance_vanilla_Xvelocity_response.^2);


IAE_f_step_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,abs(f_step_disturbance_vanilla_Xvelocity_response));
IAE_f_ramp_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,abs(f_ramp_disturbance_vanilla_Xvelocity_response));
IAE_f_sinus_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,abs(f_sinus_disturbance_vanilla_Xvelocity_response));


ITAE_f_step_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,time.*abs(f_step_disturbance_vanilla_Xvelocity_response));
ITAE_f_ramp_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,time.*abs(f_ramp_disturbance_vanilla_Xvelocity_response));
ITAE_f_sinus_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,time.*abs(f_sinus_disturbance_vanilla_Xvelocity_response));



%% forve Analysis for nn Response

ISE_f_step_disturbance_nn_Xvelocity_response = trapz(1/sim_freq,f_step_disturbance_nn_Xvelocity_response.^2);
ISE_f_ramp_disturbance_nn_Xvelocity_response = trapz(1/sim_freq,f_ramp_disturbance_nn_Xvelocity_response.^2);
ISE_f_sinus_disturbance_nn_Xvelocity_response = trapz(1/sim_freq,f_sinus_disturbance_nn_Xvelocity_response.^2);


IAE_f_step_disturbance_nn_Xvelocity_response = trapz(1/sim_freq,abs(f_step_disturbance_nn_Xvelocity_response));
IAE_f_ramp_disturbance_nn_Xvelocity_response = trapz(1/sim_freq,abs(f_ramp_disturbance_nn_Xvelocity_response));
IAE_f_sinus_disturbance_nn_Xvelocity_response = trapz(1/sim_freq,abs(f_sinus_disturbance_nn_Xvelocity_response));


ITAE_f_step_disturbance_nn_Xvelocity_response = trapz(1/sim_freq,time.*abs(f_step_disturbance_nn_Xvelocity_response));
ITAE_f_ramp_disturbance_nn_Xvelocity_response = trapz(1/sim_freq,time.*abs(f_ramp_disturbance_nn_Xvelocity_response));
ITAE_f_sinus_disturbance_nn_Xvelocity_response = trapz(1/sim_freq,time.*abs(f_sinus_disturbance_nn_Xvelocity_response));

