addpath(fullfile('..', 'src'));
addpath(fullfile('..', '@Rocket'));

close all
clear all
clc

%% General
Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controller
N = 20; % Horizon steps
H = 30; %N*Ts; % Horizon length in seconds


%% Z controller - plot with negative reference
disp('00')
mpc_z = MpcControl_z(sys_z,Ts,H)

disp('0')

% Get control input
x0_z = [0;0]; % we start at the origin
ref_z = [-4]; % we want to go to z=-4
[u_z, T_opt, X_opt, U_opt] = mpc_z.get_u(x0_z,ref_z); % ???????

disp('1')

% Plot
% I think this is the open-loop trajectory
% Evaluate once
% [u_z,T_opt,X_opt,U_opt] = mpc_z.get_u([0;0]); % [vz,z] the starting point
U_opt(:,end+1) = NaN;

% Account for linearizatioin point
X_opt = X_opt + xs([9;12]); % because xs is a long vector and we only need vz and z
U_opt = U_opt + us([3]); % we only need u_z; u=[delta1 delta2 Pavg Pdiff]

T = 10;
ph = rocket.plotvis_sub(T_opt,X_opt,U_opt,sys_z,xs,us);

disp('2')


% calculate and plot closed loop trajectory
% Tf=10; 
% x0 = [0;0]; % starting point: [vz,z]
% [T, X_sub, U_sub] = rocket.simulate_f(sys_z,x0,Tf,@mpc_z.get_u,ref_z);
% ph = rocket.plotvis_sub(T,X_sub,U_sub,sys_z,xs,us);





