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


%% Question 2:

[natural_v, natural_p] = eig(A);

p = [-1;-2;-3;-4;-5;-6]; % Desired Poles feel free to change these.
K = place(A,B,p);

% Simulating systems respose:
x_0 = [0; 0; 0; 0; 0; 0];
t = linspace(0, 240, 2400); % Modify this as needed


% Applying thrust on all motors for half time and 0 motors for half time.
u_1 = [1; 1; 1; 1]; % Reference input. Change these.
u_2 = [0; 0; 0; 0]; % Reference input. Change these.
halfway_point = 100;
U = repmat(u_1', halfway_point, 1);
U = [U; repmat(u_2', length(t) - halfway_point, 1)];

sys = ss(A, B, C, D);
% Simulate the system's response using lsim
[Y, T, X] = lsim(sys, U, t, x_0);

% Define the line width
line_width = 2;

% Define the vertical line position (time when control input was stopped)
vertical_line_time = t(halfway_point);

% Plot the response
figure;
subplot(3, 1, 1);
plot(T, X(:, 1), 'LineWidth', line_width);
title('Response of Inertial Position X');
ylabel('Inertial Position X');
xlabel('Time [S]');
line([vertical_line_time vertical_line_time], ylim, 'Color', 'r', 'LineStyle', '--');

subplot(3, 1, 2);
plot(T, X(:, 3), 'LineWidth', line_width);
title('Response of Inertial position Y');
ylabel('Inertial Position Y');
xlabel('Time [S]');
line([vertical_line_time vertical_line_time], ylim, 'Color', 'r', 'LineStyle', '--');

subplot(3, 1, 3);
plot(T, X(:, 5), 'LineWidth', line_width);
title('Response of Inertial position Z');
ylabel('Inertial Position Z');
xlabel('Time [S]');
line([vertical_line_time vertical_line_time], ylim, 'Color', 'r', 'LineStyle', '--');

figure;
subplot(3, 1, 1);
plot(T, X(:, 2), 'LineWidth', line_width);
title('Response of Inertial Velocity X');
ylabel('Inertial Velocity X');
xlabel('Time [S]');
line([vertical_line_time vertical_line_time], ylim, 'Color', 'r', 'LineStyle', '--');

subplot(3, 1, 2);
plot(T, X(:, 4), 'LineWidth', line_width);
title('Response of Inertial Velocity Y');
ylabel('Inertial Velocity Y');
xlabel('Time [S]');
line([vertical_line_time vertical_line_time], ylim, 'Color', 'r', 'LineStyle', '--');

subplot(3, 1, 3);
plot(T, X(:, 6), 'LineWidth', line_width);
title('Response of Inertial Velocity Z');
ylabel('Inertial Velocity Z');
xlabel('Time [S]');
line([vertical_line_time vertical_line_time], ylim, 'Color', 'r', 'LineStyle', '--');


%% Problem 4:
% p = [-1;-2;-3;-4;-5;-6]; % Desired Poles feel free to change these.
% K = place(A,B,p);
% F = eye(4,6);
% 
% A_cl = A - B*K;
% B_cl = B;
% sys_cl = ss(A_cl, B_cl, C, D);
% 
% % Simulating systems respose:
% x_0 = [0; 0; 0; 0; 0; 0];
% t = linspace(0, 240, 2400); % Modify this as needed
% r = [0; 0; 0; 0; 10; 0];
% for i = 1:length(t)
%     x_dot = A*x + B*u;
% end

