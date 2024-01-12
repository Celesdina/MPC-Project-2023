addpath(fullfile('..', 'src'));
%addpath(fullfile('..', '@Rocket'));
%addpath("C:\Users\leoga\Documents\MATLAB\MPC\programms\casadi-3.6.4-windows64-matlab2018b");
import casadi.*

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/40;
rocket = Rocket(Ts);

H = 4; % Horizon length in seconds
nmpc = NmpcControl(rocket, H, 7);

x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';

Tf = 2.5;
rocket.mass = 1.75;
rocket.delay = 7; % 0 if not specified, 1 = 25 ms
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph_closed = rocket.plotvis(T, X, U, Ref);
title('delay 175ms partially compensated');