clear; close all; clc

% 20 pts) Design an observer to reconstruct the state. Discuss how you determined the
% desired observer poles. Simulate the closed loop system consisting of observer and state
% variable feedback. Verify that the state observation error goes to zero at the desired
% rate. Compare the closed loop response with that obtained in Part 4
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

% Desired poles of controller
p = [-1;-1.1;-1.2;-4;-5;-6];

% Gain matrix for closed loop system which gives desired behavior via poles
K = place(A,B,p);

% If we have F matrix want to multiply it by B for full system dynamics
F = eye(size(B));

%% Building observer

 % Observer poles
op = [-1;-1.1;-1.2;-0.6;-0.1;-0.2];

% Observer gain matrix
L = place(A', C', op)';

% Combine state-feedback controller and observer
Ao  = [A-B*K B*K;
        zeros(size(A)) A-L*C];

%%%%%%%%%% TODO: UNCOMMENT WHEN WE HAVE F %%%%%%%%%%%%%%%%%%%%%
% Bo  = [B*F;
%        zeros(size(B))];
Bo  = [B;
       zeros(size(B))];

Co  = [C zeros(size(C))];

Do  = 0;
% Closed-loop observer-based control system
sys = ss(Ao, Bo, Co, Do);

%%%%%%%%%% TODO: MULTIPLY B MATRIX BY F %%%%%%%%%%%%%%%%%%%%%
sys_norm = ss(A-B*K, B, C, D);

% Simulating systems respose:
x_0 = [1; 0; 2; 0; 0; 0];
x_0_err = [1; 1; 2; 1; 1; 1];
t = linspace(0, 240, 2400); % Modify this as needed

% Applying thrust on all motors for half time and 0 motors for half time.
u_1 = [1; 1; 1; 1]; % Reference input. Change these.
u_2 = [0; 0; 0; 0]; % Reference input. Change these.
halfway_point = 800;
U = repmat(u_1', halfway_point, 1);
U = [U; repmat(u_2', length(t) - halfway_point, 1)];

% Simulate the system's response using lsim
[Y, T, X] = lsim(sys, U, t, [x_0;x_0_err]);
[Y_norm, T_norm, X_norm] = lsim(sys_norm, U, t, x_0);

% Define the line width
line_width = 2;

% Define the vertical line Position (time when control input was stopped)
vertical_line_time = t(halfway_point);

% Plot the response

x_pos = [X(:,1), X(:,3), X(:,5), X_norm(:,1), X_norm(:,3), X_norm(:,5)];
tit_pos = ["Response of Inertial Position X", "Response of Inertial Position Y", "Response of Inertial Position Z"];
xlbl_pos = repmat("Time [S]", 1, 3);
ylbl_pos = ["Inertial Position X", "Inertial Position Y", "Inertial Position Z"];
pos = custom_plot_3(T, x_pos, line_width, vertical_line_time, tit_pos, xlbl_pos, ylbl_pos);

x_pos_err = [X(:,7), X(:,9), X(:,11)];
tit_pos_err = ["Inertial Position X Estimate Error", "Inertial Position Y Estimate Error", "Inertial Position Z Estimate Error"];
xlbl_pos_err = repmat("Time [S]", 1, 3);
ylbl_pos_err = ["Error X", "Error Y", "Error Z"];
pos_err = custom_plot_3(T, x_pos_err, line_width, vertical_line_time, tit_pos_err, xlbl_pos_err, ylbl_pos_err);

x_vel = [X(:,2), X(:,4), X(:,6), X_norm(:,2), X_norm(:,4), X_norm(:,6)];
tit_vel = ["Response of Inertial Velocity X", "Response of Inertial Velocity Y", "Response of Inertial Velocity Z"];
xlbl_vel = repmat("Time [S]", 1, 3);
ylbl_vel = ["Inertial Velocity X", "Inertial Velocity Y", "Inertial Velocity Z"];
vel = custom_plot_3(T, x_vel, line_width, vertical_line_time, tit_vel, xlbl_vel, ylbl_vel);

x_vel_err = [X(:,8), X(:,10), X(:,12)];
tit_vel_err = ["Inertial Velocity X Estimate Error", "Inertial Velocity Y Estimate Error", "Inertial Velocity Z Estimate Error"];
xlbl_vel_err = repmat("Time [S]", 1, 3);
ylbl_vel_err = ["Error X", "Error Y", "Error Z"];
vel_err = custom_plot_3(T, x_vel_err, line_width, vertical_line_time, tit_vel_err, xlbl_vel_err, ylbl_vel_err);



function fig_handl = custom_plot_3(T, X, line_width, vertical_line_time, tit, xlbl, ylbl)
    
    fig_handl = figure;
    subplot(3, 1, 1);
    hold on
    plot(T, X(:, 1), 'LineWidth', line_width);
    if (size(X,2) == 6)
        plot(T, X(:, 4), 'LineWidth', line_width);
        legend('With Observer','Without Observer')
    end
    hold off
    title(tit(1));
    ylabel(ylbl(1));
    xlabel(xlbl(1));
    line([vertical_line_time vertical_line_time], ylim, 'Color', 'r', 'LineStyle', '--');
    
    subplot(3, 1, 2);
    hold on
    plot(T, X(:, 2), 'LineWidth', line_width);
    if (size(X,2) == 6)
        plot(T, X(:, 5), 'LineWidth', line_width);
        legend('With Observer','Without Observer')
    end
    hold off
    title(tit(2));
    ylabel(ylbl(2));
    xlabel(xlbl(2));
    line([vertical_line_time vertical_line_time], ylim, 'Color', 'r', 'LineStyle', '--');
    
    subplot(3, 1, 3);
    hold on
    plot(T, X(:, 3), 'LineWidth', line_width);
    if (size(X,2) == 6)
        plot(T, X(:, 6), 'LineWidth', line_width);
        legend('With Observer','Without Observer')
    end
    hold off
    title(tit(3));
    ylabel(ylbl(3));
    xlabel(xlbl(3));
    line([vertical_line_time vertical_line_time], ylim, 'Color', 'r', 'LineStyle', '--');

end















