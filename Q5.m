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
p = [-1;-2;-1;-2;-1;-2];

% Gain matrix for closed loop system which gives desired behavior via poles
K = place(A,B,p);

% If we have F matrix want to multiply it by B for full system dynamics

F = [ -18.6250    5.2849  -10.2335   63.0801   -1.9757   -3.0049;...
    4.0959  -15.3505   -9.4858   -3.6883   -9.3075   -6.8372;...
    6.1385  -14.4065  -12.7261  -35.3226   23.7870    7.3094;...
   10.2714  -13.5149    6.7720   15.0874   16.2929    8.6853];

%% Building observer

% Observer poles
op = [-4;-3;-4;-3;-4;-3];

% Observer gain matrix
L = place(A', C', op)';

% Combine state-feedback controller and observer
Ao  = [A-B*K B*K;
        zeros(size(A)) A-L*C];

%%%%%%%%%% TODO: UNCOMMENT WHEN WE HAVE F %%%%%%%%%%%%%%%%%%%%%
Bo  = [B*F;
       zeros(size(B*F))];

Co  = [C zeros(size(C))];

Do  = 0;
% Closed-loop observer-based control system
sys = ss(Ao, Bo, Co, Do);

%%%%%%%%%% TODO: MULTIPLY B MATRIX BY F %%%%%%%%%%%%%%%%%%%%%
sys_norm = ss(A-B*K, B*F, C, 0);

% Simulating systems respose:
x_0 = [0; 0; 0; 0; 0; 0];
x_0_err = [10; 0.1; 13; 0.1; 15; 0.1];
t = linspace(0, 20, 2400); % Modify this as needed

halfway_point = length(t)/2;

r_1 = [10; 0; 10; 0; 10; 0];
% r_2 = [5; 0; 5; 0; 5; 0];
R = repmat(r_1', length(t), 1);
% R = [R; repmat(r_2', length(t) - halfway_point, 1)];

% Simulate the system's response using lsim
[Y, T, X] = lsim(sys, R, t, [x_0;x_0_err]);
[Y_norm, T_norm, X_norm] = lsim(sys_norm, R, t, x_0);

% Define the line width
line_width = 2;

% Define the vertical line Position (time when control input was stopped)
% vertical_line_time = t(halfway_point);

% Plot the response

leg2 = ["With Observer","Without Observer"];

x_pos = [X(:,1), X(:,3), X(:,5), X_norm(:,1), X_norm(:,3), X_norm(:,5)];
tit_pos = ["Response of Inertial Position X", "Response of Inertial Position Y", "Response of Inertial Position Z"];
xlbl_pos = repmat("Time [S]", 1, 3);
ylbl_pos = ["Inertial Position X", "Inertial Position Y", "Inertial Position Z"];
pos = custom_plot_3(T, x_pos, line_width, tit_pos, xlbl_pos, ylbl_pos, leg2,"Impact of Observer on Inertial Position Response");

x_pos_err = [X(:,7), X(:,9), X(:,11)];
tit_pos_err = ["Inertial Position X Estimate Error", "Inertial Position Y Estimate Error", "Inertial Position Z Estimate Error"];
xlbl_pos_err = repmat("Time [S]", 1, 3);
ylbl_pos_err = ["Error X", "Error Y", "Error Z"];
pos_err = custom_plot_3(T, x_pos_err, line_width, tit_pos_err, xlbl_pos_err, ylbl_pos_err, leg2,"Inertial Position Error");

x_vel = [X(:,2), X(:,4), X(:,6), X_norm(:,2), X_norm(:,4), X_norm(:,6)];
tit_vel = ["Response of Inertial Velocity X", "Response of Inertial Velocity Y", "Response of Inertial Velocity Z"];
xlbl_vel = repmat("Time [S]", 1, 3);
ylbl_vel = ["Inertial Velocity X", "Inertial Velocity Y", "Inertial Velocity Z"];
vel = custom_plot_3(T, x_vel, line_width, tit_vel, xlbl_vel, ylbl_vel, leg2,"Impact of Observer on Inertial Velocity Response");

x_vel_err = [X(:,8), X(:,10), X(:,12)];
tit_vel_err = ["Inertial Velocity X Estimate Error", "Inertial Velocity Y Estimate Error", "Inertial Velocity Z Estimate Error"];
xlbl_vel_err = repmat("Time [S]", 1, 3);
ylbl_vel_err = ["Error X", "Error Y", "Error Z"];
vel_err = custom_plot_3(T, x_vel_err, line_width, tit_vel_err, xlbl_vel_err, ylbl_vel_err, leg2,"Inertial Velocity Error");


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
hold = ss(A,B,C,0);

alpha = 1500;
Q = alpha * eye(6);
Q(5,5) = 10;
Q(6,6) = 20;
% Q = eye(6);
R_input_6 = R;
R = eye(size(B,2));

[K6,S,P] = lqr(hold,Q,R);

% Combine state-feedback controller and observer
A6  = [A-B*K6 B*K6;
        zeros(size(A)) A-L*C];

%%%%%%%%%% TODO: UNCOMMENT WHEN WE HAVE F %%%%%%%%%%%%%%%%%%%%%
% Bo  = [B*F;
%        zeros(size(B))];
B6  = [B*F;
       zeros(size(B*F))];

C6  = [C zeros(size(C))];

D6  = 0;
% Closed-loop observer-based control system
sys6 = ss(A6, B6, C6, D6);

[Y6, T6, X6] = lsim(sys6, R_input_6, t, [x_0;x_0_err]);

%% Plots for Q6
leg6 = ["Normal Close Loop With Observer","Infinite Horizon Cost Function","Step Command"];

x_pos6 = [X(:,1), X(:,3), X(:,5), X6(:,1), X6(:,3), X6(:,5)];
tit_pos = ["Response of Inertial Position X", "Response of Inertial Position Y", "Response of Inertial Position Z"];
xlbl_pos = repmat("Time [S]", 1, 3);
ylbl_pos = ["Inertial Position X", "Inertial Position Y", "Inertial Position Z"];
pos6 = custom_plot_3(T, x_pos6, line_width, tit_pos, xlbl_pos, ylbl_pos, leg6,"Impact of Infinite Horizon Cost Function on Inertial Position Response");


%% FINDING OPTIMAL Q FOR Q6
alpha = linspace(1,2000,100);
q_5_5 = linspace(1,1000,1000);
q_6_6 = linspace(1,1000,1000);
min_err = 100;
saved_states = zeros(1,3);
for k = 1:length(alpha)
    Q = alpha(k) * eye(6);
    for i = 1:length(q_5_5)
        Q(5,5) = q_5_5(i);
        for j = 1:length(q_6_6)
            Q(6,6) = q_6_6(j);
            [K6,S,P] = lqr(hold,Q,R);
            A6  = [A-B*K6 B*K6;
            zeros(size(A)) A-L*C];
            B6  = [B*F;
                   zeros(size(B*F))];
            C6  = [C zeros(size(C))];
            D6  = 0;
            sys6 = ss(A6, B6, C6, D6);
            [Y6, T6, X6] = lsim(sys6, R_input_6, t, [x_0;x_0_err]);
            err = sqrt((X6(1200,1) - 10)^2 + (X6(1200,3) - 10)^2 + (X6(1200,5) - 10)^2);
            if(err < min_err)
                min_err = err;
                saved_states = [i,j,k];
            end
        end
    end
end

x_pos6 = [X(:,1), X(:,3), X(:,5), X6(:,1), X6(:,3), X6(:,5)];
tit_pos = ["Response of Inertial Position X", "Response of Inertial Position Y", "Response of Inertial Position Z"];
xlbl_pos = repmat("Time [S]", 1, 3);
ylbl_pos = ["Inertial Position X", "Inertial Position Y", "Inertial Position Z"];
pos6 = custom_plot_3(T, x_pos6, line_width, tit_pos, xlbl_pos, ylbl_pos, leg6,"Optimal Inf Horizon");

function fig_handl = custom_plot_3(T, X, line_width, tit, xlbl, ylbl, leg, sgtit)
    
    fig_handl = figure;
    subplot(3, 1, 1);
    hold on
    plot(T, X(:, 1), 'LineWidth', line_width);
    if (size(X,2) == 6)
        plot(T, X(:, 4), 'LineWidth', line_width);
        legend(leg(1),leg(2),'Location','SouthEast','Fontsize',16)
    end
    hold off
    title(tit(1),'Fontsize',18);
    ylabel(ylbl(1),'Fontsize',16);
    xlabel(xlbl(1),'Fontsize',16);
    
    subplot(3, 1, 2);
    hold on
    plot(T, X(:, 2), 'LineWidth', line_width);
    if (size(X,2) == 6)
        plot(T, X(:, 5), 'LineWidth', line_width);
        legend(leg(1),leg(2),'Location','SouthEast','Fontsize',16)
    end
    hold off
    title(tit(2),'Fontsize',18);
    ylabel(ylbl(2),'Fontsize',16);
    xlabel(xlbl(2),'Fontsize',16);
    
    subplot(3, 1, 3);
    hold on
    plot(T, X(:, 3), 'LineWidth', line_width);
    if (size(X,2) == 6)
        plot(T, X(:, 6), 'LineWidth', line_width);
        legend(leg(1),leg(2),'Location','SouthEast','Fontsize',16)
    end
    hold off
    title(tit(3),'Fontsize',18);
    ylabel(ylbl(3),'Fontsize',16);
    xlabel(xlbl(3),'Fontsize',16);
    sgtitle(sgtit,'Interpreter','None','Fontsize',24) 

end















