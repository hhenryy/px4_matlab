function x = myStateTransitionFcn(x,u)
% Sample time [s]
dt = 0.004;

% System Properties
m = 1;
g = 9.81;
L = 0.5;
I = 0.1;
b = 0.1;

% Using Euler discretization, next states
% can be calculated given the current
% states and input 


x = x + [ x(2); u/I-(m*g*L/I)*sin( x(1) ) - x(2)*b/I + x(3); 0]*dt; 



end