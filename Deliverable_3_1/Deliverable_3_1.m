addpath(fullfile('..', 'src'));
addpath(fullfile('..', '@Rocket'));

close all
clear all
clc


%% Deliverable 3.1 - z controller


Ts = 1/20;
rocket = Rocket(Ts);
[xs,us] = rocket.trim();
sys = rocket.linearize(xs,us);

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys,xs,us);


% Design MPc controller
H = 20; % horizon length in seconds
mpc_z = MpcControl_z(sys_z,Ts,H)

% Evaluate once
[u_z,T_opt,X_opt,U_opt] = mpc_z.get_u([0;3]); % [vz,z] the starting point
U_opt(:,end+1) = NaN;

% Account for linearizatioin point
X_opt = X_opt + xs([9;12]); % because xs is a long vector and we only need vz and z
U_opt = U_opt + us([3]); % we only need u_z; u=[delta1 delta2 Pavg Pdiff]


% calculate and plot closed loop trajectory
Tf=10; 
x0 = [0;3]; % starting point: [vz,z]
[T, X_sub, U_sub] = rocket.simulate_f(sys_z,x0,Tf,@mpc_z.get_u,0);
ph = rocket.plotvis_sub(T,X_sub,U_sub,sys_z,xs,us);

% % plot open-loop trajectory
% ph = rocket.plotvis_sub(T_opt,X_opt,U_opt+56.667,sys_z,xs,us);













