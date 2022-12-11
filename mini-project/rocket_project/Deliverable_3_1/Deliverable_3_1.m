clc;
clear;
close all;
addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
% Design MPC controller
H = 10; % Horizon length in seconds
x_indexes_list = {[2,5,7,10], [1,4,8,11], [9,12], [3,6]};                  
Tf = 10;
x0 = zeros(12,1); x0([10,11,12]) = 4; x0(6) = deg2rad(35);

%% X
mpc_x = MpcControl_x(sys_x, Ts, H);
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x0(x_indexes_list{1}));
U_opt(:,end+1) = nan;
phx = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); % Plot as usual
sgtitle("Controller of X-openloop");
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0(x_indexes_list{1}), Tf, @mpc_x.get_u, 0);
ph_x = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
sgtitle("Controller of X-closedloop");
%% Y
mpc_y = MpcControl_y(sys_y, Ts, H);
[u, T_opt, X_opt, U_opt] = mpc_y.get_u(x0(x_indexes_list{2}));
U_opt(:,end+1) = nan;
phy = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us); % Plot as usual
sgtitle("Controller of Y-openloop");
[T, Y_sub, U_sub] = rocket.simulate_f(sys_y, x0(x_indexes_list{2}), Tf, @mpc_y.get_u, 0);
ph_y = rocket.plotvis_sub(T, Y_sub, U_sub, sys_y, xs, us);
sgtitle("Controller of Y-closedloop");
%% Z
mpc_z = MpcControl_z(sys_z, Ts, H);
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(x0(x_indexes_list{3}));
U_opt(:,end+1) = nan;
phz = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us); % Plot as usual
sgtitle("Controller of Z-openloop");
[T, Z_sub, U_sub] = rocket.simulate_f(sys_z, x0(x_indexes_list{3}), Tf, @mpc_z.get_u, 0);
ph_z = rocket.plotvis_sub(T, Z_sub, U_sub, sys_z, xs, us);
sgtitle("Controller of Z-closedloop");
%% Roll
mpc_r = MpcControl_roll(sys_roll, Ts, H);
[u, T_opt, X_opt, U_opt] = mpc_r.get_u(x0(x_indexes_list{4}));
U_opt(:,end+1) = nan;
phr = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us); % Plot as usual
sgtitle("Controller of Roll-openloop");
[T, R_sub, U_sub] = rocket.simulate_f(sys_roll, x0(x_indexes_list{4}), Tf, @mpc_r.get_u, 0);
ph_r = rocket.plotvis_sub(T, R_sub, U_sub, sys_roll, xs, us);
sgtitle("Controller of Roll-closedloop");