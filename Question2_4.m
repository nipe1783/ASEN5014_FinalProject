clc 
clear
close all

%% Quadrotor Setup
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

%% Problem 2
% Simulating systems respose:

% Intial State and Time
x_0 = [0; 0; 0; 0; 0; 0];

t = linspace(0, 100, 101);

% Optimal F ?
F = [ -18.6250    5.2849  -10.2335   63.0801   -1.9757   -3.0049;...
    4.0959  -15.3505   -9.4858   -3.6883   -9.3075   -6.8372;...
    6.1385  -14.4065  -12.7261  -35.3226   23.7870    7.3094;...
   10.2714  -13.5149    6.7720   15.0874   16.2929    8.6853];

% Reference State
r1 = [10 0 10 0 10 0]';
U1 = F*r1*ones(1,51);

r2 = [5 0 5 0 5 0]';
U2 = F*r2*ones(1,50);

U = [U1,U2];

% Open Loop State Space Model
sys = ss(A, B, C, D);

% Simulate the system's response using lsim
[Y, T, X] = lsim(sys, U, t, x_0);

plotStateandOutput(T,X,Y,1,0)


%% Problem 4: Feedback Control
% Desired poles of controller
% p = [-1;-2;-1.1;-2.2;-1.2;-2.3];
p = [-1;-2;-1;-2;-1;-2];

% Gain matrix for closed loop system which gives desired behavior via poles
K = place(A,B,p);

% Same F_matrix as before 
F_opt = [ -18.6250    5.2849  -10.2335   63.0801   -1.9757   -3.0049;...
    4.0959  -15.3505   -9.4858   -3.6883   -9.3075   -6.8372;...
    6.1385  -14.4065  -12.7261  -35.3226   23.7870    7.3094;...
   10.2714  -13.5149    6.7720   15.0874   16.2929    8.6853];

% Closed Loop Dynamics
Acl = A-B*K;
Bcl = B*F_opt;

% Simulating systems respose:
% Intial State and Time
x_0 = [0; 0; 0; 0; 0; 0];
t = linspace(0, 100, 101); % Modify this as needed

% Closed Loop State Space
closedSys  = ss(Acl,Bcl,C,zeros(3,6));

% Same Reference Position as before
r1 = [10 0 10 0 10 0]';
r1 = r1.*ones(1,51);
r1 = r1';

r2 = [5 0 5 0 5 0]';
r2 = r2.*ones(1,50);
r2 = r2';

r = [r1;r2];

% Linear Simulation
[Y, T, X] = lsim(closedSys, r, t, x_0);

idx = 1;
x95 = 0.95*X(end,1:3);
for i = 1:101

    if((norm(x95 - X(i,1:3)) < 1) && (idx == 1))
        idx = i;
    end
end
plotStateandOutput(T,X,Y,2,idx)




function plotStateandOutput(T,X,Y,i, idx)
line_width = 1.5;
titlestr = ["Open Loop Control", "Closed Loop Response"];
if (i == 1)
    f1= figure;
    subplot(2,3, 1);
    plot(T, X(:, 1), 'LineWidth', line_width);
    ylabel('Inertial Position X [m]');
    xlabel('Time [S]');
    
    
    subplot(2,3, 2);
    plot(T, X(:, 3), 'LineWidth', line_width);
    ylabel('Inertial Position Y [m]');
    xlabel('Time [S]');
    
    
    subplot(2,3, 3);
    plot(T, X(:, 5), 'LineWidth', line_width);
    ylabel('Inertial Position Z [m]');
    xlabel('Time [S]');
    
    
    subplot(2,3, 4);
    plot(T, X(:, 2), 'LineWidth', line_width);
    ylabel('Inertial Velocity X [m/s]');
    xlabel('Time [S]');
    
    
    subplot(2,3, 5);
    plot(T, X(:, 4), 'LineWidth', line_width);
    ylabel('Inertial Velocity Y [m/s]');
    xlabel('Time [S]');
    
    
    subplot(2,3, 6);
    plot(T, X(:, 6), 'LineWidth', line_width);
    ylabel('Inertial Velocity Z [m/s]');
    xlabel('Time [S]');
    sgtitle(titlestr(i))
else
    f1= figure;
    subplot(2,3, 1);
    plot(T, X(:, 1), 'LineWidth', line_width);
    xline(T(idx), 'LineWidth',line_width,'Label',"95% Settling Time", 'LabelOrientation','horizontal')
    ylabel('Inertial Position X [m]');
    xlabel('Time [S]');
    
    
    subplot(2,3, 2);
    plot(T, X(:, 3), 'LineWidth', line_width);
    xline(T(idx), 'LineWidth',line_width,'Label',"95% Settling Time", 'LabelOrientation','horizontal')
    ylabel('Inertial Position Y [m]');
    xlabel('Time [S]');
    
    
    subplot(2,3, 3);
    plot(T, X(:, 5), 'LineWidth', line_width);
    xline(T(idx), 'LineWidth',line_width,'Label',"95% Settling Time", 'LabelOrientation','horizontal')
    ylabel('Inertial Position Z [m]');
    xlabel('Time [S]');
    
    
    subplot(2,3, 4);
    plot(T, X(:, 2), 'LineWidth', line_width);
    ylabel('Inertial Velocity X [m/s]');
    xlabel('Time [S]');
    
    
    subplot(2,3, 5);
    plot(T, X(:, 4), 'LineWidth', line_width);
    ylabel('Inertial Velocity Y [m/s]');
    xlabel('Time [S]');
    
    
    subplot(2,3, 6);
    plot(T, X(:, 6), 'LineWidth', line_width);
    ylabel('Inertial Velocity Z [m/s]');
    xlabel('Time [S]');
    sgtitle(titlestr(i))
end
end