

%% Disturbance frequency:
wp = 100;

wz = 50;

s = tf('s');

G__DRF = ( (s^2)/wz^2 + 1 )/( s^2/wp^2 + 1)

bode(G__DRF)