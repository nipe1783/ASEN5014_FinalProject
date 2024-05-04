clear; close all; clc

% 4. (20 pts) Design a state feedback controller to place poles in desired locations. Simulate
% the closed loop system under the same reference input used in Part 2. Discuss whether
% the closed loop response corresponds with what is expected considering the desired
% closed loop pole locations.

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
% costFunction = @(F) systemSimulation(A, B, C, D, F, K, r, t);
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

function error = systemSimulation(A, B, C, D, F, K, r, t)
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