clear all;

constants;


A11 = rho*V_trim*S*C_x_trim/m;
A12 = ((q_trim*S)/(m))*(Cl_alpha*alpha_trim+C_l_trim-(2*C_l_trim*Cl_alpha)/(pi*A*e));
A13 = 0;
A14 = g*cos(Theta_trim);

A21 = -1*rho*S*C_l_trim/m;
A22 = -q_trim*S*Cl_alpha/(m*V_trim);
A23 = 1-(q_trim*S*c_bar*Cl_q)/(m*V_bar*2*V_bar);
A24 = (-g/V_bar)*sin(Theta_trim);

A31 = 0;
A32 = (q_trim*S*c_bar)/(Iyy)*Cm_alpha;
A33 = (q_trim*S*c_bar)/(Iyy)*(c_bar)/(2*V_bar)*Cm_q;
A34 = 0;

A41 = 0;
A42 = 0;
A43 = 1;
A44 = 0;

A = [A11 A12 A13 A14; A21 A22 A23 A24; A31 A32 A33 A34; A41 A42 A43 A44]


