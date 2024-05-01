clear; close all; clc

% 2. (15 pts) Establish control system objectives in terms of desired closed loop poles and
% reference tracking accuracy. Appropriateobjectives depend on the system you have
% chosen, and should be determined by your group. Determine the system poles. Simu-
% late the system response to an appropriate reference input with no controller in place,
% and verify that this makes sense considering the system poles.

%% Setup

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

[natural_v, natural_p] = eig(A);
p = [-1;-.0104;-1;-.0104;-1;-.0208];
K = place(A,B,p);

%% Simulating System Response to reference input w/out controller.
r = [10; 1; 10; 1; 10; 1];
x_0 = [0; 0; 0; 0; 0; 0];
t = linspace(0, 240, 2400);

sys = ss(A, B, C, D);

% Finding Optimal F:
F_initial = zeros(4,6);
costFunction = @(F) systemSimulation(A, B, C, D, F, r);
options = optimset('Display', 'iter', 'PlotFcns', @optimplotfval);
F_opt = fminsearch(costFunction, F_initial, options);

% Run the system with optimal F:
u = F_opt*r;
U = repmat(u', length(t), 1);
[Y, T, X] = lsim(sys, U, t, x_0);



%% Plotting the response
line_width = 2;

figure; 
subplot(3, 1, 1);
plot(T, X(:, 1), 'LineWidth', line_width);
title('Response of Inertial Position X');
ylabel('Inertial Position X');
xlabel('Time [S]');

subplot(3, 1, 2);
plot(T, X(:, 3), 'LineWidth', line_width);
title('Response of Inertial position Y');
ylabel('Inertial Position Y');
xlabel('Time [S]');

subplot(3, 1, 3);
plot(T, X(:, 5), 'LineWidth', line_width);
title('Response of Inertial position Z');
ylabel('Inertial Position Z');
xlabel('Time [S]');

figure;
subplot(3, 1, 1);
plot(T, X(:, 2), 'LineWidth', line_width);
title('Response of Inertial Velocity X');
ylabel('Inertial Velocity X');
xlabel('Time [S]');

subplot(3, 1, 2);
plot(T, X(:, 4), 'LineWidth', line_width);
title('Response of Inertial Velocity Y');
ylabel('Inertial Velocity Y');
xlabel('Time [S]');

subplot(3, 1, 3);
plot(T, X(:, 6), 'LineWidth', line_width);
title('Response of Inertial Velocity Z');
ylabel('Inertial Velocity Z');
xlabel('Time [S]');


%% Step Response Analysis:
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

function error = systemSimulation(A, B, C, D, F, r)
    u = F * r;
    sys = ss(A, B, C, D);
    t = linspace(0, 240, 2400);
    [Y, T, X] = lsim(sys, repmat(u', length(t), 1), t);
    finalX = X(end, :);
    error = abs(finalX(2) - r(2));
    error = error + abs(finalX(4) - r(4));
    error = error + abs(finalX(6) - r(6));
end
