addpath(fullfile('..', 'src'));

%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controller
H = 30; % Horizon length in seconds
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
ref_roll= deg2rad(35);


% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state

[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, [0;0], 12, @mpc_roll.get_u, ref_roll);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, ref_roll);
