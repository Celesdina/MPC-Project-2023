addpath(fullfile('..', 'src'));

addpath(fullfile('..', '@Rocket'));
addpath("C:\Users\leoga\Documents\MATLAB\MPC\programms\casadi-3.6.4-windows64-matlab2018b");

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% Design MPC controller x
% subsys 1: sys_x 
% states: x_x = (wy, beta, vx, x)       input: u_x = (d2)

H = 30; % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);

% contorl input
x0_x = [0;0;0;0]; % we start at the origin
ref_x = [-4]; % we want to go to x=-4

% %closed loop 
Tf = 10;

[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0_x, Tf, @mpc_x.get_u, ref_x);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, ref_x);



%% Design MPC controller y
% subsys 2: sys_y
% states: x_y = (wx, alpha, vy, y)      input: u_y = (d1)

