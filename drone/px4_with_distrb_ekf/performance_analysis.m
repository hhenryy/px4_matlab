%% Performance Analysis
% Determine the performance increase of using the ESO
% Assumption: We are at steady state and a disturbance occur at 1s


%% NB NB 
% need to test the disturbance rejection of velocity and angular rate
% seperately. This has not been implemented in the Simulink model!
%% Simulation
% Duration of simulation
sim_time = 15;
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
sinus_freq = 5;

% Ramp 
ramp_slope = 0.05;
ramp_max = 0.15;
ramp_start = step_delay;

%% Setup
% Add subfolders to path
addpath(genpath('simulation_setup'));
setup_simulation;

%% Vanilla Simulation
activate_disturbance = 1;
disturbance_rejection = 0;

%% FORCE DISTURBANCES

%% Vanilla simulation, force step disturbance
option1 = 1;    % step disturbance    
option2 = 1;    % force
simulation = sim('comparison_px4_mc_sim_distrb_ekf');
% force step disturbance
force_step_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;

%% Vanilla simulation, force ramp disturbance
option1 = 2;    % ramp disturbance    
option2 = 1;    % force
simulation = sim('comparison_px4_mc_sim_distrb_ekf');
% force ramp disturbance
force_ramp_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;

%% Vanilla simulation, force sinus disturbance
option1 = 3;    % sinus disturbance    
option2 = 1;    % force
simulation = sim('comparison_px4_mc_sim_distrb_ekf');
% moment ramp disturbance
force_sinus_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;
%% EKF Simulation
disturbance_rejection = 1;
activate_disturbance = 1;

%% EKF simulation, force step disturbance
option1 = 1;    % step disturbance    
option2 = 1;    % force
simulation = sim('comparison_px4_mc_sim_distrb_ekf');
force_step_disturbance_ekf_Xvelocity_response=simulation.get('Xvelocity').Data;

%% EKF simulation, force ramp disturbance
option1 = 2;    % ramp disturbance    
option2 = 1;    % force
simulation = sim('comparison_px4_mc_sim_distrb_ekf');
force_ramp_disturbance_ekf_Xvelocity_response=simulation.get('Xvelocity').Data;
%% EKF simulation, force sinus disturbance
option1 = 3;    % sinus disturbance    
option2 = 1;    % force
simulation = sim('comparison_px4_mc_sim_distrb_ekf');
force_sinus_disturbance_ekf_Xvelocity_response=simulation.get('Xvelocity').Data;


time = simulation.get('Xvelocity').Time;


%% Analysis for vanilla response

% ISE: Integrated Squared Error
% IAE: Integrated Absolute Error
% ITAE: Integrated Time Absolute Error

ISE_force_step_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,force_step_disturbance_vanilla_Xvelocity_response.^2);
ISE_force_ramp_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,force_ramp_disturbance_vanilla_Xvelocity_response.^2);
ISE_force_sinus_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,force_sinus_disturbance_vanilla_Xvelocity_response.^2);


IAE_force_step_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,abs(force_step_disturbance_vanilla_Xvelocity_response));
IAE_force_ramp_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,abs(force_ramp_disturbance_vanilla_Xvelocity_response));
IAE_force_sinus_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,abs(force_sinus_disturbance_vanilla_Xvelocity_response));


ITAE_force_step_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,time.*abs(force_step_disturbance_vanilla_Xvelocity_response));
ITAE_force_ramp_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,time.*abs(force_ramp_disturbance_vanilla_Xvelocity_response));
ITAE_force_sinus_disturbance_vanilla_Xvelocity_response = trapz(1/sim_freq,time.*abs(force_sinus_disturbance_vanilla_Xvelocity_response));



%% Analysis for EKF Response

ISE_force_step_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,force_step_disturbance_ekf_Xvelocity_response.^2);
ISE_force_ramp_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,force_ramp_disturbance_ekf_Xvelocity_response.^2);
ISE_force_sinus_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,force_sinus_disturbance_ekf_Xvelocity_response.^2);


IAE_force_step_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,abs(force_step_disturbance_ekf_Xvelocity_response));
IAE_force_ramp_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,abs(force_ramp_disturbance_ekf_Xvelocity_response));
IAE_force_sinus_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,abs(force_sinus_disturbance_ekf_Xvelocity_response));


ITAE_force_step_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,time.*abs(force_step_disturbance_ekf_Xvelocity_response));
ITAE_force_ramp_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,time.*abs(force_ramp_disturbance_ekf_Xvelocity_response));
ITAE_force_sinus_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,time.*abs(force_sinus_disturbance_ekf_Xvelocity_response));
%% Plotting for master thesis
% Disturbance Rejection Response of Quadcopter
% Force Sinus Response
range = 10000;
range_ = range/4;
time_ = time(1:4:range);
figure(11);
plot(time_,force_step_disturbance_eso_estimation_true(1:4:range),'LineWidth',2)
hold on 
plot(time_,force_step_disturbance_eso_estimation(1:range_),'LineWidth',2)
ylabel('Disturbance Force [N]','Interpreter','latex','FontSize',12);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
legend('Ground truth', 'Estimated','Interpreter','latex','FontSize',12);
grid on

%% 
% Disturbance Rejection Response of Quadcopter
% Force Sinus Response
range = 8000;
begin= 2000;
range_ = range/4;
time_ = time(begin:4:range);
figure(104);
plot(time_,force_sinus_disturbance_ekf_estimation_true(begin:4:range),'LineWidth',2)
hold on 
plot(time_,force_sinus_disturbance_ekf_estimation(begin:4:range),'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Force [N]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin/1000, range/1000])
legend('Ground truth', 'ESO','Interpreter','latex','FontSize',fontsize,'Location','North West');
grid on
saveas(gcf,'ekf_sinus_force_rejection_estimation.pdf')
!pdfcrop ekf_sinus_force_rejection_estimation.pdf ekf_sinus_force_rejection_estimation.pdf

% Force Sinus Estimation with Rejection
range = 10000;
begin= 2000;
range_ = range/4;
time_ = time(begin:4:range);
figure(105);
plot(time_,force_sinus_disturbance_vanilla_Xvelocity_response(begin:4:range),'LineWidth',2)
hold on 
plot(time_,force_sinus_disturbance_ekf_Xvelocity_response(begin:4:range),'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Velocity [m/s]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin/1000, range/1000])
legend('PX4', 'ESO','Interpreter','latex','FontSize',fontsize);
grid on
saveas(gcf,'ekf_sinus_force_rejection_response.pdf')
!pdfcrop ekf_sinus_force_rejection_response.pdf ekf_sinus_force_rejection_response.pdf


