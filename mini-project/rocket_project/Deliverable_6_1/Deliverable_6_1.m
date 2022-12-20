clc;
clear;
close all;
addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20;
rocket = Rocket(Ts);
H = 5; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);
% MPC reference with default maximum roll = 15 deg
ref = @(t_, x_) ref_EPFL(t_);
% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) ref_EPFL(t_, roll_max);
% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x = zeros(12, 1);
x0 =x;
% [u, T_opt, X_opt, U_opt] = nmpc.get_u(x, ref);
% U_opt(:,end+1) = nan;
% ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);

%%
Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim(); 
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
rocket.anim_rate = 10;
ph = rocket.plotvis(T, X, U, Ref);