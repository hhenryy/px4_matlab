
%% pendulum constants

m = 1 ;
g = 9.81;
l = 0.1;
I = 2;
b= 0.3;

%%  PID controller
P = 3;
I = 1;
D = 0.5;


%% Disturbance

f_disturb = 2;
wd = 2*pi*f_disturb;

wp = f_disturb*2*pi;
wz = 0.1*f_disturb*2*pi;


%% Filters
s = tf('s');
zz = 0.0;
pz = 0.0;

% DRF

G__DRF = ( s^2/wz^2 +  1 )/( s^2/wp^2 +  1)
% bode(G__DRF)

%`



