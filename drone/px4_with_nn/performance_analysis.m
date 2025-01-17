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
m_step_disturbance_nn_estimation = simulation.get('torque_disturbance_est').Data;
m_step_disturbance_nn_ground_truth = simulation.get('torque_disturbance_ground_truth').Data;
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
%% Vanilla simulation, force step disturbance
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
force_step_estimation_ground_truth =simulation.get('force_disturbance_ground_truth').Data;

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
force_sinus_estimation_ground_truth =simulation.get('force_disturbance_ground_truth').Data;


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



%%
%% Plotting for Thesis Results
fontsize = 18
% Disturbance Rejection Response of Quadcopter
% Force Sinus Response
ends = 2000;
begin= 500;
range = (begin:1:ends);
figure(1);
plot(time(range),force_sinus_estimation_ground_truth(range),'LineWidth',2)
hold on 
plot(time(range),nn_force_sinus_estimation(range),'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Force [N]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin*4/1000, ends*4/1000])
legend('Ground truth', 'NN','Interpreter','latex','FontSize',fontsize,'Location','North West');
grid on
saveas(gcf,'nn_sinus_force_rejection_estimation.pdf')
!pdfcrop nn_sinus_force_rejection_estimation.pdf nn_sinus_force_rejection_estimation.pdf

% Force Sinus Estimation with Rejection
begin = 500;
ends = 3000;
range = (begin:ends);
figure(2);
plot(time(range),f_sinus_disturbance_vanilla_Xvelocity_response(range),'LineWidth',2)
hold on 
plot(time(range),f_sinus_disturbance_nn_Xvelocity_response(range),'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Velocity [m/s]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin*4/1000, ends*4/1000])
legend('PX4', 'NN','Interpreter','latex','FontSize',fontsize);
grid on
saveas(gcf,'nn_sinus_force_rejection_response.pdf')
!pdfcrop nn_sinus_force_rejection_response.pdf nn_sinus_force_rejection_response.pdf


%%
% Disturbance Rejection Response of Quadcopter
% Force Step Response
ends = 2000;
begin= 500;
range = (begin:1:ends);
figure(3);
plot(time(range),force_step_estimation_ground_truth(range),'LineWidth',2)
hold on 
plot(time(range),nn_force_step_estimation(range),'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Force [N]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin*4/1000, ends*4/1000])
legend('Ground truth', 'NN','Interpreter','latex','FontSize',fontsize,'Location','South East');
grid on
saveas(gcf,'nn_step_force_rejection_estimation.pdf')
!pdfcrop nn_step_force_rejection_estimation.pdf nn_step_force_rejection_estimation.pdf

% Force Sinus Estimation with Rejection
begin = 500;
ends = 3000;
range = (begin:ends);
figure(4);
plot(time(range),f_step_disturbance_vanilla_Xvelocity_response(range),'LineWidth',2)
hold on 
plot(time(range),f_step_disturbance_nn_Xvelocity_response(range),'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Velocity [m/s]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin*4/1000, ends*4/1000])
legend('PX4', 'NN','Interpreter','latex','FontSize',fontsize);
grid on
saveas(gcf,'nn_step_force_rejection_response.pdf')
!pdfcrop nn_step_force_rejection_response.pdf nn_step_force_rejection_response.pdf

%%
% Disturbance Rejection Response of Quadcopter
% Torque Step Response
ends = 850;
begin= 700;
range = (begin:1:ends);
figure(5);
plot(time(range),m_step_disturbance_nn_ground_truth(range),'LineWidth',2)
hold on 
plot(time(range),m_step_disturbance_nn_estimation(range),'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Torque [Nm]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin*4/1000, ends*4/1000])
legend('Ground truth', 'NN','Interpreter','latex','FontSize',fontsize,'Location','South East');
grid on
saveas(gcf,'nn_step_torque_rejection_estimation.pdf')
!pdfcrop nn_step_torque_rejection_estimation.pdf nn_step_torque_rejection_estimation.pdf

% Torque Step Estimation with Rejection
begin = 500;
ends = 1500;
range = (begin:ends);
figure(6);
plot(time(range),m_step_disturbance_vanilla_pitch_rate_response(range)*180/pi,'LineWidth',2)
hold on 
plot(time(range),m_step_disturbance_nn_pitch_rate_response(range)*180/pi,'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Pitchrate [deg/s]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin*4/1000, ends*4/1000])
legend('PX4', 'NN','Interpreter','latex','FontSize',fontsize);
grid on
saveas(gcf,'nn_step_torque_rejection_response.pdf')
!pdfcrop nn_step_torque_rejection_response.pdf nn_step_torque_rejection_response.pdf

%%
range = (1:2000);
%time = size(ekf_force_step_estimation_true
figure(7);
plot(time(range),force_sinus_disturbance_ekf_estimation_true(range),'LineWidth',2)
hold on 
plot(time(range),force_sinus_disturbance_ekf_estimation(range),'LineWidth',2)
ylabel('Disturbance force [N]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
legend('Ground truth', 'Estimated','Interpreter','latex','FontSize',fontsize);
grid on
%%
%saveas(gcf,'ekf_sinus_force_est.pdf')
%!pdfcrop ekf_sinus_force_est.pdf ekf_sinus_force_est.pdf
%%
range = (600:1000);
figure(8);
plot(time(range),moment_step_disturbance_ekf_estimation_true(range),'LineWidth',2)
hold on 
plot(time(range),moment_step_disturbance_ekf_estimation(range),'LineWidth',2)
ylabel('Disturbance Torque [Nm]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
legend('Ground truth', 'Estimated','Interpreter','latex','FontSize',fontsize);
grid on
%%
%saveas(gcf,'ekf_step_torque_est.pdf')
%!pdfcrop ekf_step_torque_est.pdf ekf_step_torque_est.pdf
%%
range = (500:1500);
figure(9);
plot(time(range),ekf_force_step_estimation_true(range),'LineWidth',2)
hold on 
plot(time(range),ekf_force_step_estimation(range),'LineWidth',2)
ylabel('Disturbance Force [N]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
legend('Ground truth', 'Estimated','Interpreter','latex','FontSize',fontsize);
grid on
%%
%saveas(gcf,'ekf_step_force_est.pdf')
%!pdfcrop ekf_step_force_est.pdf ekf_step_force_est.pdf
%%
