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

%% EKF Setup
meas_noise = 0.000001;
process_noise = 0.000001;
dstrb_process_noise = 0.001;



Qk = eye(15)*process_noise;
Rk = eye(9)*meas_noise;
Qk = eye(15)*process_noise;
Qk(10,10)=dstrb_process_noise;
Qk(11,11)=dstrb_process_noise;
Qk(12,12)=dstrb_process_noise;
Qk(13,13)=dstrb_process_noise;
Qk(14,14)=dstrb_process_noise;
Qk(15,15)=dstrb_process_noise;
Rk = eye(9)*meas_noise;

Pk_init = zeros(15);
X_hat_init = zeros(15,1);

X = X_hat_init;



P=0;
Q=0;
R=0;
U=0;
V=0;
W=0;
phi=0;
theta=0;
psi=0;


At = [
[  0,  R, -Q,                    0,                  -W,                    V,                                                 0,                                                               -g*cos(theta), 0, 1/m,   0,   0,     0,     0,     0]
[ -R,  0,  P,                    W,                   0,                   -U,                             g*cos(phi)*cos(theta),                                                      -g*sin(phi)*sin(theta), 0,   0, 1/m,   0,     0,     0,     0]
[  Q, -P,  0,                   -V,                   U,                    0,                            -g*cos(theta)*sin(phi),                                                      -g*cos(phi)*sin(theta), 0,   0,   0, 1/m,     0,     0,     0]
[  0,  0,  0,                    0, (R*(Iyy - Izz))/Ixx,  (Q*(Iyy - Izz))/Ixx,                                                 0,                                                                           0, 0,   0,   0,   0, 1/Ixx,     0,     0]
[  0,  0,  0, -(R*(Ixx - Izz))/Iyy,                   0, -(P*(Ixx - Izz))/Iyy,                                                 0,                                                                           0, 0,   0,   0,   0,     0, 1/Iyy,     0]
[  0,  0,  0,  (Q*(Ixx - Izz))/Izz, (P*(Ixx - Izz))/Izz,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0, 1/Izz]
[  0,  0,  0,                    1, sin(phi)*tan(theta),  cos(phi)*tan(theta),     Q*cos(phi)*tan(theta) - R*sin(phi)*tan(theta),               R*cos(phi)*(tan(theta)^2 + 1) + Q*sin(phi)*(tan(theta)^2 + 1), 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,            cos(phi),            -sin(phi),                         - R*cos(phi) - Q*sin(phi),                                                                           0, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0, sin(phi)/cos(theta),  cos(phi)/cos(theta), (Q*cos(phi))/cos(theta) - (R*sin(phi))/cos(theta), (R*cos(phi)*sin(theta))/cos(theta)^2 + (Q*sin(phi)*sin(theta))/cos(theta)^2, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0,     0]
[  0,  0,  0,                    0,                   0,                    0,                                                 0,                                                                           0, 0,   0,   0,   0,     0,     0,     0]
];
 
lam = [-At Qk;zeros(15), At' ];

C_noise = expm(lam*(1/sim_freq));
[col,row] =size(C_noise);

C11 = C_noise(1:col/2,1:row/2);
C12 = C_noise(1:col/2,row/2+1:end);
C21 = C_noise(col/2+1:end,1:row/2);
C22 = C_noise(col/2+1:end,row/2+1:end);
Qk = C22'*C12;

%% Vanilla Simulation
activate_disturbance = 1;
disturbance_rejection = 0;

%% FORCE DISTURBANCES

%% Vanilla simulation, force step disturbance
option1 = 1;    % step disturbance    
option2 = 1;    % force
disp("Vanilla simulation, force step disturbance");
simulation = sim('comparison_px4_mc_sim_full_distrb_ekf');
% force step disturbance
force_step_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;

%% Vanilla simulation, force ramp disturbance
option1 = 2;    % ramp disturbance    
option2 = 1;    % force
disp("Vanilla simulation, force ramp disturbance");
simulation = sim('comparison_px4_mc_sim_full_distrb_ekf');
% force ramp disturbance
force_ramp_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;

%% Vanilla simulation, force sinus disturbance
option1 = 3;    % sinus disturbance    
option2 = 1;    % force
disp("Vanilla simulation, force sinus disturbance");
simulation = sim('comparison_px4_mc_sim_full_distrb_ekf');
% moment ramp disturbance
force_sinus_disturbance_vanilla_Xvelocity_response=simulation.get('Xvelocity').Data;
%% EKF Simulation1
disturbance_rejection = 1;
activate_disturbance = 1;

%% EKF simulation, force step disturbance
option1 = 1;    % step disturbance    
option2 = 1;    % force
disp("EKF simulation, force step disturbance");
simulation = sim('comparison_px4_mc_sim_full_distrb_ekf');
force_step_disturbance_ekf_Xvelocity_response=simulation.get('Xvelocity').Data;
force_step_disturbance_ekf_Xvelocity_est = simulation.get('Xvelocity').Data;
ekf_force_step_estimation = squeeze(simulation.get('force_disturbance_estimation').Data);
ekf_force_step_estimation_true = squeeze(simulation.get('force_disturbance_true').Data);
time = simulation.get('Xvelocity').Time;
%% EKF simulation, force ramp disturbance
option1 = 2;    % ramp disturbance    
option2 = 1;    % force
disp("EKF simulation, force ramp disturbance");
simulation = sim('comparison_px4_mc_sim_full_distrb_ekf');
force_ramp_disturbance_ekf_Xvelocity_response=simulation.get('Xvelocity').Data;
%% EKF simulation, force sinus disturbance
option1 = 3;    % sinus disturbance    
option2 = 1;    % force
disp("EKF simulation, force sinus disturbance");
simulation = sim('comparison_px4_mc_sim_full_distrb_ekf');
force_sinus_disturbance_ekf_Xvelocity_response=simulation.get('Xvelocity').Data;

force_sinus_disturbance_ekf_estimation=squeeze(simulation.get('force_disturbance_estimation').Data);
force_sinus_disturbance_ekf_estimation_true=squeeze(simulation.get('force_disturbance_true').Data);


time = simulation.get('Xvelocity').Time;


%% Analysis for vanilla Velocity Disturbance response

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



%% Analysis for EKF Velocity Disturbance Response

ISE_force_step_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,force_step_disturbance_ekf_Xvelocity_response.^2);
ISE_force_ramp_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,force_ramp_disturbance_ekf_Xvelocity_response.^2);
ISE_force_sinus_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,force_sinus_disturbance_ekf_Xvelocity_response.^2);


IAE_force_step_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,abs(force_step_disturbance_ekf_Xvelocity_response));
IAE_force_ramp_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,abs(force_ramp_disturbance_ekf_Xvelocity_response));
IAE_force_sinus_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,abs(force_sinus_disturbance_ekf_Xvelocity_response));


ITAE_force_step_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,time.*abs(force_step_disturbance_ekf_Xvelocity_response));
ITAE_force_ramp_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,time.*abs(force_ramp_disturbance_ekf_Xvelocity_response));
ITAE_force_sinus_disturbance_ekf_Xvelocity_response = trapz(1/sim_freq,time.*abs(force_sinus_disturbance_ekf_Xvelocity_response));


%% Vanilla Simulation
activate_disturbance = 1;
disturbance_rejection = 0;

%% MOMENT DISTURBANCES

%% Vanilla simulation, moment step disturbance
option1 = 1;    % step disturbance    
option2 = 2;    % moment
disp("Vanilla simulation, moment step disturbance");
simulation = sim('comparison_px4_mc_sim_full_distrb_ekf');
% force step disturbance
moment_step_disturbance_vanilla_pitchrate_response=simulation.get('pitchrate').Data;

%% Vanilla simulation, moment ramp disturbance
option1 = 2;    % ramp disturbance    
option2 = 2;    % moment
disp("Vanilla simulation, moment ramp disturbance");
simulation = sim('comparison_px4_mc_sim_full_distrb_ekf');
% force ramp disturbance
moment_ramp_disturbance_vanilla_pitchrate_response=simulation.get('pitchrate').Data;

%% Vanilla simulation, moment sinus disturbance
option1 = 3;    % sinus disturbance    
option2 = 2;    % force
disp("Vanilla simulation, moment sinus disturbance");
simulation = sim('comparison_px4_mc_sim_full_distrb_ekf');
% moment ramp disturbance
moment_sinus_disturbance_vanilla_pitchrate_response=simulation.get('pitchrate').Data;
%% EKF Simulation1
disturbance_rejection = 1;
activate_disturbance = 1;

%% EKF simulation, moment step disturbance
option1 = 1;    % step disturbance    
option2 = 2;    % moment
disp("EKF simulation, moment step disturbance");
simulation = sim('comparison_px4_mc_sim_full_distrb_ekf');
moment_step_disturbance_ekf_pitchrate_response=simulation.get('pitchrate').Data;

moment_step_disturbance_ekf_estimation= squeeze(simulation.get('torque_disturbance_estimation').Data);
moment_step_disturbance_ekf_estimation_true=squeeze(simulation.get('torque_disturbance_true').Data);

%% EKF simulation, moment ramp disturbance
option1 = 2;    % ramp disturbance    
option2 = 2;    % moment
disp("EKF simulation, moment ramp disturbance");
simulation = sim('comparison_px4_mc_sim_full_distrb_ekf');
moment_ramp_disturbance_ekf_pitchrate_response=simulation.get('pitchrate').Data;
%% EKF simulation, moment sinus disturbance
option1 = 3;    % sinus disturbance    
option2 = 2;    % moment
disp("EKF simulation, moment sinus disturbance");
simulation = sim('comparison_px4_mc_sim_full_distrb_ekf');
moment_sinus_disturbance_ekf_pitchrate_response=simulation.get('pitchrate').Data;


time = simulation.get('Xvelocity').Time;

%% Analysis for vanilla moment Disturbance response

% ISE: Integrated Squared Error
% IAE: Integrated Absolute Error
% ITAE: Integrated Time Absolute Error

ISE_moment_step_disturbance_vanilla_pitchrate_response = trapz(1/sim_freq,moment_step_disturbance_vanilla_pitchrate_response.^2);
ISE_moment_ramp_disturbance_vanilla_pitchrate_response = trapz(1/sim_freq,moment_ramp_disturbance_vanilla_pitchrate_response.^2);
ISE_moment_sinus_disturbance_vanilla_pitchrate_response = trapz(1/sim_freq,moment_sinus_disturbance_vanilla_pitchrate_response.^2);


IAE_moment_step_disturbance_vanilla_pitchrate_response = trapz(1/sim_freq,abs(moment_step_disturbance_vanilla_pitchrate_response));
IAE_moment_ramp_disturbance_vanilla_pitchrate_response = trapz(1/sim_freq,abs(moment_ramp_disturbance_vanilla_pitchrate_response));
IAE_moment_sinus_disturbance_vanilla_pitchrate_response = trapz(1/sim_freq,abs(moment_sinus_disturbance_vanilla_pitchrate_response));


ITAE_moment_step_disturbance_vanilla_pitchrate_response = trapz(1/sim_freq,time.*abs(moment_step_disturbance_vanilla_pitchrate_response));
ITAE_moment_ramp_disturbance_vanilla_pitchrate_response = trapz(1/sim_freq,time.*abs(moment_ramp_disturbance_vanilla_pitchrate_response));
ITAE_moment_sinus_disturbance_vanilla_pitchrate_response = trapz(1/sim_freq,time.*abs(moment_sinus_disturbance_vanilla_pitchrate_response));



%% Analysis for EKF moment Disturbance Response

ISE_moment_step_disturbance_ekf_pitchrate_response = trapz(1/sim_freq,moment_step_disturbance_ekf_pitchrate_response.^2);
ISE_moment_ramp_disturbance_ekf_pitchrate_response = trapz(1/sim_freq,moment_ramp_disturbance_ekf_pitchrate_response.^2);
ISE_moment_sinus_disturbance_ekf_pitchrate_response = trapz(1/sim_freq,moment_sinus_disturbance_ekf_pitchrate_response.^2);


IAE_moment_step_disturbance_ekf_pitchrate_response = trapz(1/sim_freq,abs(moment_step_disturbance_ekf_pitchrate_response));
IAE_moment_ramp_disturbance_ekf_pitchrate_response = trapz(1/sim_freq,abs(moment_ramp_disturbance_ekf_pitchrate_response));
IAE_moment_sinus_disturbance_ekf_pitchrate_response = trapz(1/sim_freq,abs(moment_sinus_disturbance_ekf_pitchrate_response));


ITAE_moment_step_disturbance_ekf_pitchrate_response = trapz(1/sim_freq,time.*abs(moment_step_disturbance_ekf_pitchrate_response));
ITAE_moment_ramp_disturbance_ekf_pitchrate_response = trapz(1/sim_freq,time.*abs(moment_ramp_disturbance_ekf_pitchrate_response));
ITAE_moment_sinus_disturbance_ekf_pitchrate_response = trapz(1/sim_freq,time.*abs(moment_sinus_disturbance_ekf_pitchrate_response));

%% Plotting for Thesis Results
fontsize = 18
% Disturbance Rejection Response of Quadcopter
% Force Sinus Response
ends = 2000;
begin= 500;
range = (begin:1:ends);
figure(1);
plot(time(range),force_sinus_disturbance_ekf_estimation_true(range),'LineWidth',2)
hold on 
plot(time(range),force_sinus_disturbance_ekf_estimation(range),'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Force [N]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin*4/1000, ends*4/1000])
legend('Ground truth', 'EKF','Interpreter','latex','FontSize',fontsize,'Location','North West');
grid on
saveas(gcf,'ekf_sinus_force_rejection_estimation.pdf')
!pdfcrop ekf_sinus_force_rejection_estimation.pdf ekf_sinus_force_rejection_estimation.pdf

% Force Sinus Estimation with Rejection
begin = 500;
ends = 3000;
range = (begin:ends);
figure(2);
plot(time(range),force_sinus_disturbance_vanilla_Xvelocity_response(range),'LineWidth',2)
hold on 
plot(time(range),force_sinus_disturbance_ekf_Xvelocity_response(range),'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Velocity [m/s]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin*4/1000, ends*4/1000])
legend('PX4', 'EKF','Interpreter','latex','FontSize',fontsize);
grid on
saveas(gcf,'ekf_sinus_force_rejection_response.pdf')
!pdfcrop ekf_sinus_force_rejection_response.pdf ekf_sinus_force_rejection_response.pdf


%%
% Disturbance Rejection Response of Quadcopter
% Force Step Response
ends = 2000;
begin= 500;
range = (begin:1:ends);
figure(3);
plot(time(range),ekf_force_step_estimation_true(range),'LineWidth',2)
hold on 
plot(time(range),ekf_force_step_estimation(range),'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Force [N]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin*4/1000, ends*4/1000])
legend('Ground truth', 'EKF','Interpreter','latex','FontSize',fontsize,'Location','South East');
grid on
saveas(gcf,'ekf_step_force_rejection_estimation.pdf')
!pdfcrop ekf_step_force_rejection_estimation.pdf ekf_step_force_rejection_estimation.pdf

% Force Sinus Estimation with Rejection
begin = 500;
ends = 3000;
range = (begin:ends);
figure(4);
plot(time(range),force_step_disturbance_vanilla_Xvelocity_response(range),'LineWidth',2)
hold on 
plot(time(range),force_step_disturbance_ekf_Xvelocity_response(range),'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Velocity [m/s]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin*4/1000, ends*4/1000])
legend('PX4', 'EKF','Interpreter','latex','FontSize',fontsize);
grid on
saveas(gcf,'ekf_step_force_rejection_response.pdf')
!pdfcrop ekf_step_force_rejection_response.pdf ekf_step_force_rejection_response.pdf

%%
% Disturbance Rejection Response of Quadcopter
% Torque Step Response
ends = 850;
begin= 700;
range = (begin:1:ends);
figure(5);
plot(time(range),moment_step_disturbance_ekf_estimation_true(range),'LineWidth',2)
hold on 
plot(time(range),moment_step_disturbance_ekf_estimation(range),'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Torque [Nm]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin*4/1000, ends*4/1000])
legend('Ground truth', 'EKF','Interpreter','latex','FontSize',fontsize,'Location','South East');
grid on
saveas(gcf,'ekf_step_torque_rejection_estimation.pdf')
!pdfcrop ekf_step_torque_rejection_estimation.pdf ekf_step_torque_rejection_estimation.pdf

% Torque Step Estimation with Rejection
begin = 500;
ends = 1500;
range = (begin:ends);
figure(6);
plot(time(range),moment_step_disturbance_vanilla_pitchrate_response(range)*180/pi,'LineWidth',2)
hold on 
plot(time(range),moment_step_disturbance_ekf_pitchrate_response(range)*180/pi,'LineWidth',2)
ax = gca;
ax.YAxis.FontSize = 16; %for y-axis 
ax.XAxis.FontSize = 16; %for y-axis
ylabel('Pitchrate [deg/s]','Interpreter','latex','FontSize',fontsize);
xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
xlim([begin*4/1000, ends*4/1000])
legend('PX4', 'EKF','Interpreter','latex','FontSize',fontsize);
grid on
saveas(gcf,'ekf_step_torque_rejection_response.pdf')
!pdfcrop ekf_step_torque_rejection_response.pdf ekf_step_torque_rejection_response.pdf

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
saveas(gcf,'ekf_sinus_force_est.pdf')
!pdfcrop ekf_sinus_force_est.pdf ekf_sinus_force_est.pdf
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
saveas(gcf,'ekf_step_torque_est.pdf')
!pdfcrop ekf_step_torque_est.pdf ekf_step_torque_est.pdf
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
saveas(gcf,'ekf_step_force_est.pdf')
!pdfcrop ekf_step_force_est.pdf ekf_step_force_est.pdf
%%


% %% Plotting for Thesis Results
% range = (1:2000);
% figure(11);
% plot(time(range),force_step_disturbance_ekf_estimation_true(range),'LineWidth',2)
% hold on 
% plot(time(range),force_step_disturbance_ekf_estimation(range),'LineWidth',2)
% ylabel('Disturbance force [N]','Interpreter','latex','FontSize',fontsize);
% xlabel('Time [s]','Interpreter','latex','FontSize',fontsize);
% legend('Ground truth', 'Estimated','Interpreter','latex','FontSize',fontsize);
% grid on
% 
% %% Plotting graphs of interest
% figure(1)
% plot(force_step_disturbance_ekf_Xvelocity_response,'LineWidth',2)
% hold on
% %plot(force_step_disturbance_vanilla_Xvelocity_response(4:4:end),'LineWidth',2)
% %plot(force_step_disturbance_eso_Xvelocity_response(4:4:end),'LineWidth',2)
% %plot(f_step_disturbance_nn_Xvelocity_response,'LineWidth',2)
% title('Response of quadcopter experiencing a force step disturbance','Interpreter','latex','FontSize',20)
% ylabel('Velocity - [m/s]','Interpreter','latex','FontSize',fontsize);
% xlabel('Timestep','Interpreter','latex','FontSize',fontsize);
% legend({'EKF','PX4','ESO','NNO',},'Interpreter','latex','FontSize',fontsize)
% grid on
% 
% 
% %% Plotting graphs of interest
% figure(22)
% plot(moment_step_disturbance_ekf_pitchrate_response,'LineWidth',2)
% hold on
% plot(moment_step_disturbance_vanilla_pitchrate_response,'LineWidth',2)
% plot(moment_step_disturbance_eso_pitch_rate_response(4:4:end),'LineWidth',2)
% plot(m_step_disturbance_nn_pitch_rate_response,'LineWidth',2)
% title('Response of quadcopter experiencing a torque step disturbance','Interpreter','latex','FontSize',20)
% ylabel('Pitchrate - [rad/s]','Interpreter','latex','FontSize',12,'FontSize',20);
% xlabel('Timestep','Interpreter','latex','FontSize',12,'FontSize',20);
% legend({'EKF','PX4','ESO','NNO',},'Interpreter','latex','FontSize',20)
% grid on
% 
% 
% %% Plotting graphs of interest
% eso_force_step_estimation = eso_force_step_estimation(:,:);
% figure(3)
% plot(ekf_force_step_estimation(:,:),'LineWidth',2)
% hold on
% plot(eso_force_step_estimation(4:4:end),'LineWidth',2)
% grid on
% plot(nn_force_step_estimation,'LineWidth',2)
% 
% title('Estimation of force disturbance','Interpreter','latex','FontSize',20)
% ylabel('Force - [N]','Interpreter','latex','FontSize',12,'FontSize',20);
% xlabel('Timestep','Interpreter','latex','FontSize',12,'FontSize',20);
% legend({'EKF','ESO','NNO',},'Interpreter','latex','FontSize',20)
% 
% 
% %% Plotting graphs of interest
% eso_force_sinus_estimation = eso_force_sinus_estimation(:,:);
% figure(3)
% plot(ekf_force_sinus_estimation(:,:),'LineWidth',2)
% hold on
% plot(eso_force_sinus_estimation(4:4:end),'LineWidth',2)
% grid on
% plot(nn_force_sinus_estimation,'LineWidth',2)
% 
% title('Estimation of force disturbance','Interpreter','latex','FontSize',20)
% ylabel('Force - [N]','Interpreter','latex','FontSize',12,'FontSize',20);
% xlabel('Timestep','Interpreter','latex','FontSize',12,'FontSize',20);
% legend({'EKF','ESO','NNO',},'Interpreter','latex','FontSize',20)
% 
