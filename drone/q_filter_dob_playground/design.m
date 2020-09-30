%% Frequency based design for disturbance rejection
% Designing a Q filter for a 2nd order minimum phase system

%% Script setup
s = tf('s');


%% 2nd Order System
zeta = 0.01;
wn = 1; 

G_ol = 1/(s^2 +2*wn*zeta*s + wn^2);

% figure(1);
% bode(G_ol)
% grid on


%% Q Filter: Low Pass Filter
w_c = 5;

Q = 1/(1/w_c*s +1);

% figure(2)
% bode(Q)
% grid on