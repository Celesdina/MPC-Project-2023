addpath(fullfile('..', 'src'));


close all
clear all
clc

%% trim and decompose into 4 sub-systems
Ts = 1/20; % sample time
rocket = Rocket(Ts);
[xs,us] = rocket.trim();
sys = rocket.linearize(xs,us);

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys,xs,us);



%% Deliverable 3.1 - x controller

% states: x_x = (wy, beta, vx, x)       input: u_x = (d2)

H_x = 30; % Horizon length in seconds 
mpc_x = MpcControl_x(sys_x, Ts, H_x);
% Get control input ( x is the index of the subsystem here)
u_x = mpc_x.get_u([0 0 0 3]');

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u_x, T_opt_x, X_opt_x, U_opt_x] = mpc_x.get_u([0 0 0 3]');
U_opt_x(:,end+1) = NaN;

% % Account for linearization point (xs, us)%
X_opt_x =  X_opt_x + xs([2, 5, 7, 10]);
U_opt_x = U_opt_x + us(2);

%open loop
plot_open_loop_x = rocket.plotvis_sub(T_opt_x, X_opt_x, U_opt_x, sys_x, xs, us);
title('open-loop: x');

%closed loop 
Tf = 10;
[T_x, X_sub_x, U_sub_x] = rocket.simulate_f(sys_x, [0 0 0 3]', Tf, @mpc_x.get_u, 0);
plot_closed_loop_x = rocket.plotvis_sub(T_x, X_sub_x, U_sub_x, sys_x, xs, us);
title('closed-loop: x');



%% Deliverable 3.1 - y controller

% states: x_y = (wx, alpha, vy, y)      input: u_y = (d1)
H_y = 30; % Horizon length in seconds 
mpc_y = MpcControl_y(sys_y, Ts, H_y);
% Get control input ( x is the index of the subsystem here)
u_y = mpc_y.get_u([0 0 0 3]');

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u_y, T_opt_y, X_opt_y, U_opt_y] = mpc_y.get_u([0 0 0 3]');
U_opt_y(:,end+1) = NaN;

% Account for linearization point (xs, us)%
X_opt_y =  X_opt_y + xs([1, 4, 8, 11]);
U_opt_y = U_opt_y + us(1);

% open loop
plot_open_loop_y = rocket.plotvis_sub(T_opt_y, X_opt_y, U_opt_y, sys_y, xs, us); 
title('open-loop: y');


%closed loop 
Tf_y = 10;
[T_y, X_sub_y, U_sub_y] = rocket.simulate_f(sys_y, [0 0 0 3]', Tf_y, @mpc_y.get_u, 0);
plot_closed_loop_y = rocket.plotvis_sub(T_y, X_sub_y, U_sub_y, sys_y, xs, us);
title('closed-loop: y');


%% Deliverable 3.1 - z controller

% Design MPc controller
H_z = 30; % horizon length in seconds
mpc_z = MpcControl_z(sys_z,Ts,H_z); 

% Evaluate once
[u_z,T_opt_z,X_opt_z,U_opt_z] = mpc_z.get_u([0;3]); % [vz,z] the starting point
U_opt_z(:,end+1) = NaN;

% Account for linearization point
X_opt_z = X_opt_z + xs([9;12]); % xs[9]=vz, xs[12]=z
U_opt_z = U_opt_z + us([3]); % u=[delta1 delta2 Pavg Pdiff]

% plot open-loop trajectory
plot_open_loop_z = rocket.plotvis_sub(T_opt_z,X_opt_z,U_opt_z,sys_z,xs,us);
title("plot open-loop: z")

% calculate and plot closed loop trajectory
Tf_z=10; 
x0_z = [0;3]; % starting point: [vz,z]
[T_z, X_sub_z, U_sub_z] = rocket.simulate_f(sys_z,x0_z,Tf_z,@mpc_z.get_u,0);
plot_closed_loop_z = rocket.plotvis_sub(T_z,X_sub_z,U_sub_z,sys_z,xs,us);
title("plot closed-loop: z")


%% Deliverable 3.1 - roll controller

% Design MPC controller
H_roll = 30; % Horizon length in seconds
mpc_roll = MpcControl_roll(sys_roll, Ts, H_roll);


% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u_roll, T_opt_roll, X_opt_roll, U_opt_roll] = mpc_roll.get_u([0;deg2rad(30)]);
U_opt_roll(:,end+1) = NaN;

% Account for linearization point
X_opt_roll = X_opt_roll+xs([3,6]);
U_opt_roll = U_opt_roll+us([4]);

% open-loop plot
plot_open_loop_roll = rocket.plotvis_sub(T_opt_roll, X_opt_roll, U_opt_roll, sys_roll, xs, us); 
title("open-loop: roll");

% closed-loop plot
[T_roll, X_sub_roll, U_sub_roll] = rocket.simulate_f(sys_roll, [0;deg2rad(30)], 12, @mpc_roll.get_u, 0);
plot_closed_loop_roll = rocket.plotvis_sub(T_roll, X_sub_roll, U_sub_roll, sys_roll, xs, us);
title("closed-loop: roll");



