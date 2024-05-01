clc
clear all


k_dLat = .0104; % N*s/m
k_dVert = .0208; % N*s/m
m = 1; %kg
delta = .04167;
gamma = .4;

A = [0 1 0 0 0 0;
     0, -k_dLat/m 0 0 0 0;
     0 0 0 1 0 0;
     0 0 0 -k_dLat/m  0 0;
     0 0 0 0 0 1;
     0 0 0 0 0 -k_dVert/m];

B = [0 0 0 0;
     -delta/m 0 delta/m 0;
     0 0 0 0;
     0 -delta/m 0 delta/m;
     0 0 0 0;
     gamma/m gamma/m gamma/m gamma/m];

C = [1, 0, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0;
    0, 0, 0, 0, 1, 0];

D = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];


%% Question 4:

[natural_v, natural_p] = eig(A);

p = [-1;-2;-3;-4;-5;-6]; % Desired Poles feel free to change these.
K = place(A,B,p);
A_cl = A - B*K;
B_cl = B;
sys_cl = ss(A_cl, B_cl, C, D);


% Simulating systems respose:
x_0 = [0; 0; 0; 0; 0; 0];
t = linspace(0, 240, 2400); % Modify this as needed

r = [0; 0; 0; 0; 10; 0];
[Y, T, X] = lsim(sys_cl, r, t, x_0);
