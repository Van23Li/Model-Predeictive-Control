clear 
close all
addpath(fullfile('..', 'src'));
addpath(fullfile('..', 'mosek'));
%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
% Compute steady−state for which 0 = f(xs,us)
[xs, us] = rocket.trim();
% Linearize the nonlinear model about trim point
sys = rocket.linearize(xs, us);
% Compute the four independent systems
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Set state
% x = [w, phi, v, p]'; 
% w:    1~3, the angular velocities about the body axes
% phi:  4~6, the Euler angles
% v:    7~9, the velocity
% p:  10~12, the position
state_indexes_list = {[2,5,7,10],
                  [1,4,8,11],
                  [9,12],
                  [3,6]};                  
Tf = 10;
state0 = zeros(12,1); state0([10,11,12]) = 5; state0(6) = pi/4;

% %% X
% % Design MPC controller
% H = 5; % Horizon length in seconds 
% mpc_x = MpcControl_x(sys_x, Ts, H);
% 
% % Get control input
% % u_x = mpc_x.get_u(state0);
% 
% % Open−loop
% [u, T_opt, X_opt, U_opt] = mpc_x.get_u(state0(state_indexes_list{1}));
% U_opt(:,end+1) = nan;
% ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); % Plot as usual
% 
% % Close-loop
% [T, X_sub, U_sub] = rocket.simulate_f(sys_x, state0(state_indexes_list{1}), Tf, @mpc_x.get_u, 0);
% ph_x = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
% 
% %% Y
% % Design MPC controller
% H = 5; % Horizon length in seconds 
% mpc_y = MpcControl_y(sys_y, Ts, H);
% 
% % Get control input
% % u_x = mpc_x.get_u(state0);
% 
% % Open−loop
% [u, T_opt, Y_opt, U_opt] = mpc_y.get_u(state0(state_indexes_list{2}));
% U_opt(:,end+1) = nan;
% ph = rocket.plotvis_sub(T_opt, Y_opt, U_opt, sys_y, xs, us); % Plot as usual
% 
% % Close-loop
% [T, Y_sub, U_sub] = rocket.simulate_f(sys_y, state0(state_indexes_list{1}), Tf, @mpc_y.get_u, 0);
% ph_y = rocket.plotvis_sub(T, Y_sub, U_sub, sys_y, xs, us);

%% Z
% Design MPC controller
H = 5; % Horizon length in seconds 
mpc_z = MpcControl_z(sys_z, Ts, H);

% Get control input
% u_x = mpc_x.get_u(state0);

% Open−loop
[u, T_opt, Z_opt, U_opt] = mpc_z.get_u(state0(state_indexes_list{3}));
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, Z_opt, U_opt, sys_z, xs, us); % Plot as usual

% Close-loop
[T, Z_sub, U_sub] = rocket.simulate_f(sys_z, state0(state_indexes_list{3}), Tf, @mpc_z.get_u, 0);
ph_z = rocket.plotvis_sub(T, Z_sub, U_sub, sys_z, xs, us);

%% Roll
% Design MPC controller
H = 5; % Horizon length in seconds 
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Get control input
% u_x = mpc_x.get_u(state0);

% Open−loop
[u, T_opt, Roll_opt, U_opt] = mpc_roll.get_u(state0(state_indexes_list{4}));
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, Roll_opt, U_opt, sys_roll, xs, us); % Plot as usual

% Close-loop
[T, Roll_sub, U_sub] = rocket.simulate_f(sys_roll, state0(state_indexes_list{4}), Tf, @mpc_roll.get_u, 0);
ph_roll = rocket.plotvis_sub(T, Roll_sub, U_sub, sys_roll, xs, us);