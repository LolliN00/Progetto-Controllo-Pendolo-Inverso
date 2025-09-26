clc;    
clear;  
close all; 
a = 500; %filtro derivatore
A = [
    0, 0, 1, 0;
    0, 0, 0, 1;
    0, 35.7650, -56.51923, 0;
    0, 36.43262, -19.45591, 0
];
B = [
    0;
    0;
    47.54433;
    16.36643
];
C = [1 0 0 0;0 1 0 0];
D = [0,0]';
sys = ss(A,B,C,D);
theta_1sp = pi/4;
desired_poles = [-2.6+1.2*1i, -2.6-1.2*1i, -20, -25];
K = place(A, B, desired_poles);
% Discretizzazione del filtro derivatore
Ts = 0.0001;
num_d = [a -a];
den_d = [1 Ts*a-1];

