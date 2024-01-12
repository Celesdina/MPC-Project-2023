addpath(fullfile('..', 'src'));

close all
clc
clear all

%% 
rocket = Rocket(Ts);
Tf = 5.0;

% system definition x0: w, phi, v, p
x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]';
% input u: d1, d2, Pavg, Pdiff
u = [deg2rad([0 0]), 63, 0]';

[T,X,U] = rocket.simulate(x0,Tf,u);

rocket.anim_rate = 1.0;
rocket.vis(T,X,U);




%% NOTES

% Situation 1:
% x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 5]';
% u = [deg2rad([0 0]), 80, 0]';
% the rocket goes up straight and stabilizes at a height depending on Pavg
% if Pavg = 20 (e.g.), z < 0 -> weird, shouldn't be possible -> that's
% cause there's no controller yet

% Descend vertically without tipping over:
% system definition x0: w, phi, v, p
% x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 -10, 0 0 10]';
% input u: d1, d2, Pavg, Pdiff
% u = [deg2rad([0 0]), 80, 0]';

% Rotation around z axis:
% system definition x0: w, phi, v, p
% x0 = [deg2rad([0 0 500, 0 0 0]), 0 0 5, 0 0 0]';
% input u: d1, d2, Pavg, Pdiff
% u = [deg2rad([0 0]), 80, 0]';

% Fly along the x axis:% system definition x0: w, phi, v, p
% x0 = [deg2rad([0 0 0, 0 90 0]), 0 0 5, 0 0 0]';
% input u: d1, d2, Pavg, Pdiff
% u = [deg2rad([0 0]), 80, 0]';
% it flies a parabola cause there's a 

% Hover in space:
% system definition x0: w, phi, v, p
% x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]';
% input u: d1, d2, Pavg, Pdiff
% u = [deg2rad([0 0]), 63, 0]';


