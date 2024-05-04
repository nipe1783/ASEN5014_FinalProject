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
r = [10; 0; 10; 0; 10; 0];
x_0 = [0; 0; 0; 0; 0; 0];
t = linspace(0, 240, 2400);

sys = ss(A, B, C, D);

% Finding Optimal F:
% F_initial = zeros(4,6);
% costFunction = @(F) systemSimulation2(A, B, C, D, F, r);
% options = optimset('Display', 'iter', 'PlotFcns', @optimplotfval,'TolX', 1e-10,'TolFun', 1e-10);
% F_opt = fminsearch(costFunction, F_initial, options);
F_opt = [-0.558198290031071	-0.794523027382378	-0.266287214719760	-1.47135807177475	-0.289746218373586	-2.45572944427126
0.0232719454848269	2.37504263859915	-0.891016842284312	0.386208007616082	-0.144159483802471	-0.0409672415159455
0.353814473239561	1.30411459550084	0.316434375667081	-0.105843440193003	0.0613825521664272	0.162452773214766
0.446331282328694	-1.41659591472375	0.954099006126827	-0.692405077797945	0.278798522055473	2.45891824523758];

% Run the system with optimal F:
u = F_opt*r;
U = repmat(u', length(t), 1);
[Y, T, X] = lsim(sys, U, t, x_0);
plotStateandOutput(T,X,Y)


% Step Response Analysis:
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


%% Problem 4: Feedback Control

% Desired poles of controller
p = [-1;-.1;-1;-.1;-1;-.1];
% Gain matrix for closed loop system which gives desired behavior via poles
K = place(A,B,p);
r = [10 0 10 0 10 0]';


% Simulating systems respose:
x_0 = [0; 0; 0; 0; 0; 0];
t = linspace(0, 240, 2400);
dt = t(2)-t(1);


% COMMENT THIS OUT IF YOU DONT WANT TO FIND A NEW F
% F_initial = zeros(4,6);
% costFunction = @(F) systemSimulation4(A, B, C, D, F, K, r, t);
% options = optimset('Display', 'iter', 'PlotFcns', @optimplotfval,'TolX', 1e-10,'TolFun', 1e-10, 'MaxFunEvals', 1e10);
% F_opt = fminsearch(costFunction, F_initial, options);
F_opt = [-0.558198290031071	-0.794523027382378	-0.266287214719760	-1.47135807177475	-0.289746218373586	-2.45572944427126
0.0232719454848269	2.37504263859915	-0.891016842284312	0.386208007616082	-0.144159483802471	-0.0409672415159455
0.353814473239561	1.30411459550084	0.316434375667081	-0.105843440193003	0.0613825521664272	0.162452773214766
0.446331282328694	-1.41659591472375	0.954099006126827	-0.692405077797945	0.278798522055473	2.45891824523758];

% Linear Simulation
x = x_0;
y = C*x;
for i = 1:length(t)-1
    u(:,i) = F_opt*r - K*x(:,i); % New control
    dX(:,i+1) = A*x(:,i) + B*u(:,i); % dX with new control
    x(:,i+1) = x(:,i) + dX(:,i+1)*dt; % next linear state
    y(:,i+1) = C*x(:,i+1); % next output
end
X = x';
Y = y';
plotStateandOutput(t,X,Y)


function plotStateandOutput(T,X,Y)

line_width = 1.5;

figure;
subplot(2,3, 1);
plot(T, X(:, 1), 'LineWidth', line_width);
title('Response of Inertial Position X');
ylabel('Inertial Position X');
xlabel('Time [S]');


subplot(2,3, 2);
plot(T, X(:, 3), 'LineWidth', line_width);
title('Response of Inertial position Y');
ylabel('Inertial Position Y');
xlabel('Time [S]');


subplot(2,3, 3);
plot(T, X(:, 5), 'LineWidth', line_width);
title('Response of Inertial position Z');
ylabel('Inertial Position Z');
xlabel('Time [S]');


subplot(2,3, 4);
plot(T, X(:, 2), 'LineWidth', line_width);
title('Response of Inertial Velocity X');
ylabel('Inertial Velocity X');
xlabel('Time [S]');


subplot(2,3, 5);
plot(T, X(:, 4), 'LineWidth', line_width);
title('Response of Inertial Velocity Y');
ylabel('Inertial Velocity Y');
xlabel('Time [S]');


subplot(2,3, 6);
plot(T, X(:, 6), 'LineWidth', line_width);
title('Response of Inertial Velocity Z');
ylabel('Inertial Velocity Z');
xlabel('Time [S]');

end

function error = systemSimulation4(A, B, C, D, F, K, r, t)
    x_0 = [0; 0; 0; 0; 0; 0];
    dt = t(2) - t(1);
    x = x_0;
    y = C * x;
    for i = 1:length(t)-1
        u(:,i) = F * r - K * x(:,i); % New control
        dX(:,i+1) = A * x(:,i) + B * u(:,i); % dX with new control
        x(:,i+1) = x(:,i) + dX(:,i+1) * dt; % next linear state
        y(:,i+1) = C * x(:,i+1); % next output
    end
    X = x';
    error = 0;
    for i = 1:length(t)
        error = error + sum((r - X(i, :)').^2);
    end
end

function error = systemSimulation2(A, B, C, D, F, r)
    u = F * r;
    sys = ss(A, B, C, D);
    t = linspace(0, 240, 2400);
    [Y, T, X] = lsim(sys, repmat(u', length(t), 1), t);
    error = 0;
    for i = 1:length(t)
        error = error + sum((r - X(i, :)').^2);
    end
end
