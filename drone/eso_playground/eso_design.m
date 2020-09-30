%% Designing a Extended State Observor for Linear Pendulum
zeta = 0.1;
wn = 1; 

%% A Matrix

a11 = 0;
a12 = 1;
a13 = 0;

a21 = 0;
a22 = 0;
a23 = 1;

a31 = 0;
a32 = -wn^2;
a33 = -2*zeta*wn;

A = [a11 a12 a13; a21 a22 a23; a31 a32 a33];

%% B Matrix

B = [0;1;0];


%% C & D Matrix

C = [1 0 0];
D = 0;

%% desired pole location
% text book poles
% poles_control = [0.4, -2,-2];
% poles_eso = [-67.4695 0.0124 -7.1429];


poles_control = [-2*wn -2*wn -2*wn];
poles_eso = 10*poles_control;

K = acker(A,B,poles_control);
L = acker(A',C',poles_eso)';

[cl_b,cl_a] = ss2tf(A-B*K,B,C,D);
SYS_CLOSED = tf(cl_b,cl_a);

[comp_b,comp_a] = ss2tf(A-L*C,B,C,D);
SYS_COMP = tf(comp_b,comp_a);



wd = 10;

L = [3*wd 3*wd^2 wd^3]';


