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
p = [-1.1;-2.1;-3.9;-2.15;-1.4;-2.3];

% Gain matrix for closed loop system which gives desired behavior via poles
K = place(A,B,p);

% If we have F matrix want to multiply it by B for full system dynamics
% F = ones(4,6);
% F(3:4,:) = 2*F(3:4,:)

F = [-132.064937876818	-70.6494618657608	-0.883893838673886	14.5688168059468	33.0453371555401	-203.206432594426
72.2335696400482	101.090592395416	-101.539359701207	-22.3946356904322	-7.22370006795745	203.551453011367
-8.11602707473124	-140.204038704589	106.532674433090	-43.8591736745134	-10.9154203015058	38.3158635623288
-9.90389510334388	6.84564867828146	97.9221078354695	70.6088072047673	-47.4835413225389	187.103412971369];

%% Building observer

 % Observer poles
op = [-1;-1.1;-1.2;-0.6;-0.1;-0.2];

% Observer gain matrix
L = place(A', C', op)';

% Combine state-feedback controller and observer
Ao  = [A-B*K B*K;
        zeros(size(A)) A-L*C];

%%%%%%%%%% TODO: UNCOMMENT WHEN WE HAVE F %%%%%%%%%%%%%%%%%%%%%
Bo  = [B*F;
       zeros(size(B*F))];
% Bo  = [B;
%        zeros(size(B))];

Co  = [C zeros(size(C))];

Do  = 0;
% Closed-loop observer-based control system
sys = ss(Ao, Bo, Co, Do);

%%%%%%%%%% TODO: MULTIPLY B MATRIX BY F %%%%%%%%%%%%%%%%%%%%%
sys_norm = ss(A-B*K, B*F, C, 0);

% Simulating systems respose:
x_0 = [1; 1; 2; 1; 1; 1];
x_0_err = [1; 1; 2; 1; 1; 1];
t = linspace(0, 240, 2400); % Modify this as needed

% Applying thrust on all motors for half time and 0 motors for half time.
u_1 = [1; 1; 1; 1]; % Reference input. Change these.
u_2 = [0; 0; 0; 0]; % Reference input. Change these.
halfway_point = length(t)/2;
U = repmat(u_1', halfway_point, 1);
U = [U; repmat(u_2', length(t) - halfway_point, 1)];

r_1 = [1000; 1; 10; 1; 10; 1];
r_2 = [5; 1; 5; 1; 5; 1];
R = repmat(r_1', halfway_point, 1);
R = [R; repmat(r_2', length(t) - halfway_point, 1)];

% Simulate the system's response using lsim
[Y, T, X] = lsim(sys, R, t, [x_0;x_0_err]);
[Y_norm, T_norm, X_norm] = lsim(sys_norm, R, t, x_0);

% Define the line width
line_width = 2;

% Define the vertical line Position (time when control input was stopped)
vertical_line_time = t(halfway_point);

% Plot the response

leg = ["With Observer","Without Observer"];

x_pos = [X(:,1), X(:,3), X(:,5), X_norm(:,1), X_norm(:,3), X_norm(:,5)];
tit_pos = ["Response of Inertial Position X", "Response of Inertial Position Y", "Response of Inertial Position Z"];
xlbl_pos = repmat("Time [S]", 1, 3);
ylbl_pos = ["Inertial Position X", "Inertial Position Y", "Inertial Position Z"];
pos = custom_plot_3(T, x_pos, line_width, vertical_line_time, tit_pos, xlbl_pos, ylbl_pos, leg);

x_pos_err = [X(:,7), X(:,9), X(:,11)];
tit_pos_err = ["Inertial Position X Estimate Error", "Inertial Position Y Estimate Error", "Inertial Position Z Estimate Error"];
xlbl_pos_err = repmat("Time [S]", 1, 3);
ylbl_pos_err = ["Error X", "Error Y", "Error Z"];
pos_err = custom_plot_3(T, x_pos_err, line_width, vertical_line_time, tit_pos_err, xlbl_pos_err, ylbl_pos_err, leg);

x_vel = [X(:,2), X(:,4), X(:,6), X_norm(:,2), X_norm(:,4), X_norm(:,6)];
tit_vel = ["Response of Inertial Velocity X", "Response of Inertial Velocity Y", "Response of Inertial Velocity Z"];
xlbl_vel = repmat("Time [S]", 1, 3);
ylbl_vel = ["Inertial Velocity X", "Inertial Velocity Y", "Inertial Velocity Z"];
vel = custom_plot_3(T, x_vel, line_width, vertical_line_time, tit_vel, xlbl_vel, ylbl_vel, leg);

x_vel_err = [X(:,8), X(:,10), X(:,12)];
tit_vel_err = ["Inertial Velocity X Estimate Error", "Inertial Velocity Y Estimate Error", "Inertial Velocity Z Estimate Error"];
xlbl_vel_err = repmat("Time [S]", 1, 3);
ylbl_vel_err = ["Error X", "Error Y", "Error Z"];
vel_err = custom_plot_3(T, x_vel_err, line_width, vertical_line_time, tit_vel_err, xlbl_vel_err, ylbl_vel_err, leg);


%% Q6

% 10 pts) Develop an infinite-horizon cost function, and solve for the corresponding
% optimal state feedback law. Where are the closed loop poles located? Implement
% this (with an observer for the state) in simulation, and compare the response to that
% obtained in Part 5.

% Weight for state impact on cost function
% As alpha goes up the states have more impact on the cost so we work to
% drive them down with a fast response at the expese of control effort
%
% As alpha goes down the control effort is more important so we have a slow
% response but use little control effort
%
% For a drone we want fast response or else we crash and we can use lots of
% control effort
alpha = 10000;
Q = alpha * eye(size(A));

R = eye(size(B,2));

[K6,S,P] = lqr(sys_norm,Q,R);

% Combine state-feedback controller and observer
A6  = [A-B*K6 B*K6;
        zeros(size(A)) A-L*C];

%%%%%%%%%% TODO: UNCOMMENT WHEN WE HAVE F %%%%%%%%%%%%%%%%%%%%%
% Bo  = [B*F;
%        zeros(size(B))];
B6  = [B;
       zeros(size(B))];

C6  = [C zeros(size(C))];

D6  = 0;
% Closed-loop observer-based control system
sys6 = ss(A6, B6, C6, D6);

[Y6, T6, X6] = lsim(sys6, U, t, [x_0;x_0_err]);

%% Plots for Q6
leg6 = ["Normal Close Loop With Observer","Infinite Horizon Cost Function"];

x_pos6 = [X(:,1), X(:,3), X(:,5), X6(:,1), X6(:,3), X6(:,5)];
tit_pos = ["Response of Inertial Position X", "Response of Inertial Position Y", "Response of Inertial Position Z"];
xlbl_pos = repmat("Time [S]", 1, 3);
ylbl_pos = ["Inertial Position X", "Inertial Position Y", "Inertial Position Z"];
pos6 = custom_plot_3(T, x_pos6, line_width, vertical_line_time, tit_pos, xlbl_pos, ylbl_pos, leg6);




function fig_handl = custom_plot_3(T, X, line_width, vertical_line_time, tit, xlbl, ylbl, leg)
    
    fig_handl = figure;
    subplot(3, 1, 1);
    hold on
    plot(T, X(:, 1), 'LineWidth', line_width);
    if (size(X,2) == 6)
        plot(T, X(:, 4), 'LineWidth', line_width);
        legend(leg(1),leg(2))
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
        legend(leg(1),leg(2))
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
        legend(leg(1),leg(2))
    end
    hold off
    title(tit(3));
    ylabel(ylbl(3));
    xlabel(xlbl(3));
    line([vertical_line_time vertical_line_time], ylim, 'Color', 'r', 'LineStyle', '--');

end















