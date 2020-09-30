%% constants

Ts = 0.004; % 250Hz

%% Noise Profiles
process_theta_variance = 0;
process_theta_dot_variance = 0;
process_disturbance_variance = 0;

measure_theta_variance = 0;
measure_theta_dot_variance = 0;
measure_disturbance_variance = 0;

Qk = eye(2)*[process_theta_variance;process_theta_dot_variance]*Ts

Vk =  eye(2)*[measure_theta_variance;measure_theta_dot_variance];




syms theta theta_dot m g L I b u f
func = [theta_dot, -(m*g*L/I)*sin(theta)-b/I*theta_dot+u/I+f/I,0];
vars = [theta, theta_dot,f];

x = jacobian(func, vars)

z = eye(3) + x*Ts






