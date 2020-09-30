%%  Constants
m = 2;
g = 9.81;
L = 1;
I = 2;
b = 0.5;
Ts = 0.004;
sim_freq = 0.004;

meas_theta_var = 0.0001;
meas_theta_dot_var = 0.0001;
process_theta_var = 0.001;
process_theta_dot_var = 0.001;

Qk = [process_theta_var 0 ;0 process_theta_dot_var];
Rk = [meas_theta_var 0; 0 meas_theta_dot_var];

Pk_init = zeros(2);
X_hat_init = [0;0];


At = [ 0 1; -L*g*m/I -b/I];

lam = [-At Qk;zeros(2), At' ];

C = expm(lam*Ts)
[col,row] =size(C);

C11 = C(1:col/2,1:row/2);
C12 = C(1:col/2,row/2+1:end);
C21 = C(col/2+1:end,1:row/2);
C22 = C(col/2+1:end,row/2+1:end);


Qk = C22'*C12;

% Qk = [process_theta_var 0 ;0 process_theta_dot_var];


