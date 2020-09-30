%%  Constants
format long
m = 1;
g = 9.81;
L = 0.5;
I = 0.1;
b = 0.1;
Ts = 0.004;
sim_freq = 0.004;

meas_theta_var = 0.000001;
meas_theta_dot_var = 0.000001;
process_theta_var = 0.0001;
process_theta_dot_var = 0.0001;

process_disturbance_var = 0.001;

Qk = [process_theta_var 0 0;0 process_theta_dot_var 0; 0 0 process_disturbance_var] ;
Rk = [meas_theta_var 0 ; 0 meas_theta_dot_var];

Pk_init = zeros(3);
X_hat_init = [0;0;0];


At = [ 0 1 0; -L*g*m/I -b/I -1/I; 0 0 0];
Ht = [1 0 0; 0 1 0];
%%

Ob = obsv(At,Ht);

unob = length(At) - rank(Ob)
%%
lam = [-At Qk;zeros(3), At' ];

C = expm(lam*Ts)
[col,row] =size(C);

C11 = C(1:col/2,1:row/2);
C12 = C(1:col/2,row/2+1:end);
C21 = C(col/2+1:end,1:row/2);
C22 = C(col/2+1:end,row/2+1:end)'


Qk = C22'*C12

% Qk(end,:) = 0
% Qk(end,end) = 0.000001
% Qk = [process_theta_var 0 ;0 process_theta_dot_var];

