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
[ux, T_optx, X_optx, U_optx] = mpc_x.get_u(x0(x_indexes_list{1}));
U_optx(:,end+1) = nan;
phx = rocket.plotvis_sub(T_optx, X_optx, U_optx, sys_x, xs, us); % Plot as usual
sgtitle("Controller of X-openloop");
[Tx, X_subx, U_subx] = rocket.simulate_f(sys_x, x0(x_indexes_list{1}), Tf, @mpc_x.get_u, 0);
ph_x = rocket.plotvis_sub(Tx, X_subx, U_subx, sys_x, xs, us);
sgtitle("Controller of X-closedloop");
%% Y
mpc_y = MpcControl_y(sys_y, Ts, H);
[uy, T_opty, X_opty, U_opty] = mpc_y.get_u(x0(x_indexes_list{2}));
U_opty(:,end+1) = nan;
phy = rocket.plotvis_sub(T_opty, X_opty, U_opty, sys_y, xs, us); % Plot as usual
sgtitle("Controller of Y-openloop");
[Ty, Y_suby, U_suby] = rocket.simulate_f(sys_y, x0(x_indexes_list{2}), Tf, @mpc_y.get_u, 0);
ph_y = rocket.plotvis_sub(Ty, Y_suby, U_suby, sys_y, xs, us);
sgtitle("Controller of Y-closedloop");
%% Z
mpc_z = MpcControl_z(sys_z, Ts, H);
[uz, T_optz, X_optz, U_optz] = mpc_z.get_u(x0(x_indexes_list{3}));
U_optz(:,end+1) = nan;
phz = rocket.plotvis_sub(T_optz, X_optz, U_optz, sys_z, xs, us); % Plot as usual
sgtitle("Controller of Z-openloop");
[Tz, Z_subz, U_subz] = rocket.simulate_f(sys_z, x0(x_indexes_list{3}), Tf, @mpc_z.get_u, 0);
ph_z = rocket.plotvis_sub(Tz, Z_subz, U_subz, sys_z, xs, us);
sgtitle("Controller of Z-closedloop");
%% Roll
mpc_r = MpcControl_roll(sys_roll, Ts, H);
[ur, T_optr, X_optr, U_optr] = mpc_r.get_u(x0(x_indexes_list{4}));
U_optr(:,end+1) = nan;
phr = rocket.plotvis_sub(T_optr, X_optr, U_optr, sys_roll, xs, us); % Plot as usual
sgtitle("Controller of Roll-openloop");
[Tr, R_subr, U_subr] = rocket.simulate_f(sys_roll, x0(x_indexes_list{4}), Tf, @mpc_r.get_u, 0);
ph_r = rocket.plotvis_sub(Tr, R_subr, U_subr, sys_roll, xs, us);
sgtitle("Controller of Roll-closedloop");