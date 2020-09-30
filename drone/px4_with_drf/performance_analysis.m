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

% Sinusoid
sinus_amplitude = 0.2;
sinus_bias = 0;
sinus_freq = 3;


%% DRF
%% DRF: Disturbance Rejection Filter


f_disturb = sinus_freq;

wp = f_disturb;
wz = 1;

%% Filters
s = tf('s');
zz = 10;
pz = 1;

% DRF
G__DRF = ( s^2/wz^2 +  1 )/( s^2/wp^2 +  1);
discrete_DRF = c2d(G__DRF,1/sim_freq);

%% Setup
% Add subfolders to path
addpath(genpath('simulation_setup'));
setup_simulation;
ROS2_delay=2.5/sim_freq;    %% Communcication of 100 Hz

%% Vanilla Simulation
activate_disturbance = 1;
disturbance_rejection = 0;
angular_rejection = 0;
velocity_rejection = 0;

%% Vanilla simulation, sinus moment disturbance
disp("Vanilla simulation, moment sinus disturbance");
option1 = 3;    % sinus disturbance    
option2 = 2;    % moment
simulation = sim('px4_mc_drf_comparisons_sim');
% moment ramp disturbance
m_sinus_disturbance_vanilla_pitch_rate_response=simulation.get('pitchrate').Data;
m_sinus_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;
%% nn Simulation
disturbance_rejection = 1;
activate_disturbance = 1;
angular_rejection = 1;
velocity_rejection = 0;

%% nn simulation, moment sinus disturbance
disp("nn simulation, moment sinus disturbance");
option1 = 3;    % sinus disturbance    
option2 = 2;    % moment
simulation = sim('px4_mc_drf_comparisons_sim');
m_sinus_disturbance_nn_pitch_rate_response=simulation.get('pitchrate').Data;
m_sinus_disturbance_nn_Xvelocity_response=simulation.get('Xvelocity').Data;


time = simulation.get('pitchrate').Time;

%% FORCE SIMULATION VANILLA
disturbance_rejection = 0;
activate_disturbance = 1;
angular_rejection = 0;
velocity_rejection = 0;

%% Vanilla simulation, force force disturbance
disp("Vanilla simulation, force sinus disturbance");
option1 = 3;    % sinus disturbance    
option2 = 1;    % force
simulation = sim('px4_mc_drf_comparisons_sim');
% moment ramp disturbance
f_sinus_disturbance_vanilla_pitch_rate_response=simulation.get('pitchrate').Data;
f_sinus_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;

%% Neural networks FORCE SIMULATION
disturbance_rejection = 1;
activate_disturbance = 1;
angular_rejection = 0;
velocity_rejection = 1;
%% nn simulation, force sinus disturbance
disp("nn simulation, force sinus disturbance");
option1 = 3;    % sinus disturbance    
option2 = 1;    % force
simulation = sim('px4_mc_drf_comparisons_sim');
f_sinus_disturbance_nn_pitch_rate_response=simulation.get('pitchrate').Data;
f_sinus_disturbance_nn_Xvelocity_response=simulation.get('Xvelocity').Data;


time = simulation.get('pitchrate').Time;

%% Moment Analysis for vanilla response

% ISE: Integrated Squared Error
% IAE: Integrated Absolute Error
% ITAE: Integrated Time Absolute Error


ISE_m_sinus_disturbance_vanilla_pitch_rate_response = trapz(1/sim_freq,m_sinus_disturbance_vanilla_pitch_rate_response.^2);



IAE_m_sinus_disturbance_vanilla_pitch_rate_response = trapz(1/sim_freq,abs(m_sinus_disturbance_vanilla_pitch_rate_response));



ITAE_m_sinus_disturbance_vanilla_pitch_rate_response = trapz(1/sim_freq,time.*abs(m_sinus_disturbance_vanilla_pitch_rate_response));



%% moment Analysis for nn Response


ISE_m_sinus_disturbance_nn_pitch_rate_response = trapz(1/sim_freq,m_sinus_disturbance_nn_pitch_rate_response.^2);



IAE_m_sinus_disturbance_nn_pitch_rate_response = trapz(1/sim_freq,abs(m_sinus_disturbance_nn_pitch_rate_response));



ITAE_sinus_disturbance_nn_pitch_rate_response = trapz(1/sim_freq,time.*abs(m_sinus_disturbance_nn_pitch_rate_response));

%% %% force Analysis for vanilla response

% ISE: Integrated Squared Error
% IAE: Integrated Absolute Error
% ITAE: Integrated Time Absolute Error


ISE_f_sinus_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,f_sinus_disturbance_vanilla_Xvelocity_response.^2);


IAE_f_sinus_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,abs(f_sinus_disturbance_vanilla_Xvelocity_response));


ITAE_f_sinus_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,time.*abs(f_sinus_disturbance_vanilla_Xvelocity_response));



%% forve Analysis for nn Response

ISE_f_sinus_disturbance_nn_Xvelocity_response = trapz(1/sim_freq,f_sinus_disturbance_nn_Xvelocity_response.^2);


IAE_f_sinus_disturbance_nn_Xvelocity_response = trapz(1/sim_freq,abs(f_sinus_disturbance_nn_Xvelocity_response));


ITAE_f_sinus_disturbance_nn_Xvelocity_response = trapz(1/sim_freq,time.*abs(f_sinus_disturbance_nn_Xvelocity_response));

