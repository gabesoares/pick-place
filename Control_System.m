% Feedback Control System for Pick and Place Robot
% Proportional controller

%% System parameters
K_i = 2.18e-2; % Nm/A Torque constant
R_a = 4.33; % Ohms Armature resistance
L = 2.34e-3; % H Inductance
K_b = 2.18e-2; % V/(rad/s) Back-emf constant

J_m = 1.6e-6; % Kgm^2 Motor inertia
B_m = 1.1e-4; % N*m*s damping constant

J_L = 0; % Kgm^2 Truss + magnet inertia
n = 3; % Motor gearbox gear ratio
J = J_L/n^2 + J_m; % Kgm^2 Total system inertia

K_s = 1; % sensor gain from angle to voltage.. can be combined with Kp

K = 1; % Kp for proportional controller

%% System Transfer Function
% Assuming inductance << armature resistance, the system can be modelled
% using a second order transfer function.

w2 = K*K_i*K_s/(R_a*J);
zw2 = (R_a*B_m + K_i*K_b)/(R_a*J);

G = tf(w2, [1 zw2 w2]);