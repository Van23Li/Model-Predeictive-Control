addpath(genpath('mini-project'));
import casadi.*
x = MX.sym('x')
disp(jacobian(sin(x),x))


%%

Ts = 1/20;

rocket = Rocket(Ts);
Tf = 2.0;

x0 = [deg2rad([2 -2 0, -2 2 0]), 0 0 0, 0 0 0]';
u  = [deg2rad([2 0]), 60, 0]';
[T, X, U] = rocket.simulate(x0, Tf, u);

rocket.anim_rate = 1.0;
rocket.vis(T, X, U);