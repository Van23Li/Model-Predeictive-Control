clc;
clear;
close all;
addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
Ts = 1 / 20;
rocket = Rocket(Ts); 
[xs, us] = rocket.trim(); 
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 10;
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
%%
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
%%
Tf = 30;
x0 = zeros(12,1);
ref = @(t_, x_) ref_EPFL(t_);
% tune mass, both feasible, mass 2.0 clearer impacts
rocket.mass = 1.8; % Manipulate mass for simulation
% rocket.mass = 2.0; % Manipulate mass for simulation
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
%%
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
%%
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation_est_z'; % Set a figure title
% %%
% plot(Z_hat(end,11:end));