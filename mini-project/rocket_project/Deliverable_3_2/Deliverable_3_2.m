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

H = 10; % Horizon length in seconds 
x_indexes_list = {[2,5,7,10], [1,4,8,11], [9,12], [3,6]};                  
Tf = 10;
x0 = zeros(12,1); 
%%
mpc_x = MpcControl_x(sys_x, Ts, H);
x_ref = -4;
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0(x_indexes_list{1}), Tf, @mpc_x.get_u, x_ref);
ph_x = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_ref);
%% 
mpc_y = MpcControl_y(sys_y, Ts, H);
y_ref = -4;
[T, Y_sub, U_sub] = rocket.simulate_f(sys_y, x0(x_indexes_list{2}), Tf, @mpc_y.get_u, y_ref);
ph_y = rocket.plotvis_sub(T, Y_sub, U_sub, sys_y, xs, us, y_ref);
%% 
mpc_z = MpcControl_z(sys_z, Ts, H);
z_ref = -4;
[T, Z_sub, U_sub] = rocket.simulate_f(sys_z, x0(x_indexes_list{3}), Tf, @mpc_z.get_u, z_ref);
ph_z = rocket.plotvis_sub(T, Z_sub, U_sub, sys_z, xs, us, z_ref);
%% 
mpc_r = MpcControl_roll(sys_roll, Ts, H);
r_ref = deg2rad(35);  
[T, R_sub, U_sub] = rocket.simulate_f(sys_roll, x0(x_indexes_list{4}), Tf, @mpc_r.get_u, r_ref);
ph_r = rocket.plotvis_sub(T, R_sub, U_sub, sys_roll, xs, us, r_ref);