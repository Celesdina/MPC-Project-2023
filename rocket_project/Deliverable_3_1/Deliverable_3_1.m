addpath(fullfile('..', 'src'));
addpath(fullfile('..', '@Rocket'));
addpath("C:\Users\leoga\Documents\MATLAB\MPC\programms\casadi-3.6.4-windows64-matlab2018b");

%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable
close all
clear all
clc
% subsys 1: sys_x 
% states: x_x = (wy, beta, vx, x)       input: u_x = (d2)

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% Design MPC controller x
H = 10; % Horizon length in seconds % i chose 20 with balck magic
mpc_x = MpcControl_x(sys_x, Ts, H);
% Get control input ( x is the index of the subsystem here)
u_x = mpc_x.get_u([0 0 0 3]');

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_x.get_u([0 0 0 3]');
U_opt(:,end+1) = NaN;

% % Account for linearization point (xs, us)%
X_opt =  X_opt + xs([2, 5, 7, 10]);
U_opt = U_opt + us(2);

%open loop
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us);
title('open loop x');


%closed loop 
Tf = 10;
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, [0 0 0 3]', Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
title('closed loop x');



%% Design MPC controller y
% subsys 2: sys_y
% states: x_y = (wx, alpha, vy, y)      input: u_y = (d1)
H = 10; % Horizon length in seconds % i chose 10 with balck magic
mpc_y = MpcControl_y(sys_y, Ts, H);
% Get control input ( x is the index of the subsystem here)
u_y = mpc_y.get_u([0 0 0 3]');

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_y.get_u([0 0 0 3]');
U_opt(:,end+1) = NaN;

% % Account for linearization point (xs, us)%
X_opt =  X_opt + xs([1, 4, 8, 11]);
U_opt = U_opt + us(1);

% open loop
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us); 
title('open loop y');


%closed loop 
Tf = 10;
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, [0 0 0 3]', Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);
title('closed loop y');
