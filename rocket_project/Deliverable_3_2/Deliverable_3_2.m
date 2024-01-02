addpath(fullfile('..', 'src'));

%close all
clear all
%clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controllers
H = 20; % Horizon length in seconds

ref_x=-4;
ref_y=-4;
ref_z=-4;
ref_roll= deg2rad(35);

%x
mpc_x = MpcControl_x(sys_x, Ts, H);

%open-loop
[u_x, T_optx, X_optx, U_optx] = mpc_x.get_u([0;0;0;0],ref_x);
U_optx(:,end+1) = NaN;

X_optx =  X_optx + xs([2, 5, 7, 10]);
U_optx = U_optx + us(2);
ph = rocket.plotvis_sub(T_optx, X_optx, U_optx, sys_x, xs, us);
title("X open-loop")

%closed-loop
[Tx, X_x, U_x] = rocket.simulate_f(sys_x, [0;0;0;0], 10, @mpc_x.get_u, ref_x);
phx = rocket.plotvis_sub(Tx, X_x, U_x, sys_x, xs, us, ref_x);
title("X closed-loop")

%y
mpc_y = MpcControl_y(sys_y, Ts, H);

%open-loop
[u_y, T_opty, X_opty, U_opty] = mpc_y.get_u([0;0;0;0], ref_y);
U_opty(:,end+1) = NaN;

X_opty =  X_opty + xs([1, 4, 8, 11]);
U_opty = U_opty + us(1);
ph = rocket.plotvis_sub(T_opty, X_opty, U_opty, sys_y, xs, us);
title("Y open-loop");

%closed-loop
[Ty, X_y, U_y] = rocket.simulate_f(sys_y, [0;0;0;0], 10, @mpc_y.get_u, ref_y);
phy = rocket.plotvis_sub(Ty, X_y, U_y, sys_y, xs, us, ref_y);
title("Y closed-loop");

%z
mpc_z = MpcControl_z(sys_z, Ts, H);

%open-loop
[u_z, T_optz, X_optz, U_optz] = mpc_z.get_u([0;0], ref_z);
U_optz(:,end+1) = NaN;

X_optz = X_optz + xs([9;12]); % because xs is a long vector and we only need vz and z
U_optz = U_optz + us([3]); % we only need u_z; u=[delta1 delta2 Pavg Pdiff]
ph = rocket.plotvis_sub(T_optz, X_optz, U_optz, sys_z, xs, us);
title("Z open-loop");

%closed-loop
[Tz, X_z, U_z] = rocket.simulate_f(sys_z, [0;0], 10, @mpc_z.get_u, ref_z);
phz = rocket.plotvis_sub(Tz, X_z, U_z, sys_z, xs, us, ref_z);
title("Z closed-loop");

%roll
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

%open-loop
[u_r, T_optr, X_optr, U_optr] = mpc_roll.get_u([0;0],ref_roll);
U_optr(:,end+1) = NaN;

X_optr = X_optr+xs([3,6]);
U_optr = U_optr+us([4]);
ph = rocket.plotvis_sub(T_optr, X_optr, U_optr, sys_roll, xs, us);
title("Roll open-loop");

%closed-loop
[Tr, X_r, U_r] = rocket.simulate_f(sys_roll, [0;0], 10, @mpc_roll.get_u, ref_roll);
phr = rocket.plotvis_sub(Tr, X_r, U_r, sys_roll, xs, us, ref_roll);
title("Roll closed-loop");
