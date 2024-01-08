addpath(fullfile('..', 'src'));
addpath(fullfile('..', '@Rocket'));
addpath("C:\Users\leoga\Documents\MATLAB\MPC\programms\casadi-3.6.4-windows64-matlab2018b");
import casadi.*


%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20;
rocket = Rocket(Ts);

H = 5; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);

% MPC reference with default maximum roll = 15 deg
ref = @(t_, x_) ref_TVC(t_);
% MPC reference with specified maximum roll = 50 deg
% roll_max = deg2rad(50);
% ref = @(t_, x_) ref_TVC(t_, roll_max);
% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state

x1 = zeros(12,1);
ref4 = [10 10 10 deg2rad(15)]';

[u, T_opt, X_opt, U_opt] = nmpc.get_u(x1, ref4);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref4);
title('open loop');

Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
% ph = rocket.plotvis(T, X, U, Ref);
% title('closed loop');
