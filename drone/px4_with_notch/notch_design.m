%% Design notch filter for 5Hz


s = tf('s');

wc = 5;
o = 2;


notch = (s^2 + wc^2)/(s^2 + o*s + wc^2);

bode(notch)