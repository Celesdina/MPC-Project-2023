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


% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state

[u, T_opt, X_opt, U_opt] = mpc_roll.get_u([0;deg2rad(30)]);
U_opt(:,end+1) = NaN;

% Account for linearization point
X_opt = X_opt+xs([3,6]);
U_opt = U_opt+us([4]);
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us); % Plot as usual

[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, [0;deg2rad(30)], 12, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);


%:)
