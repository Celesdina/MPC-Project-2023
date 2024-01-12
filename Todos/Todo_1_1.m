addpath(fullfile('..', 'src'));

% d1, d2: deflection angles of servo 1 and 2 about x_b and y_b
% Pavg = (P1+P2)/2
% Pdiff = P2-P1

% input
d1 = 0;
d2 = 0;
Pavg = 20;
Pdiff = 0;

% system definition
w = [0;0;2*pi];
phi = [0.5;0.5;0.5];
v = [1;1;1];
p = [1;1;1];

Ts = 1/20;
rocket = Rocket(Ts);
u = [d1, d2, Pavg, Pdiff]';
[b_f,b_M] = rocket.getForceAndMomentFromThrust(u)
%x = [w,phi,v,p]';
x = [0 0 6,0.5 0.5 0.5, 0 0 0, 0 0 0]';
%x = [deg2rad([2 -2 0, -2 2 0]), 0 0 0, 0 0 0]';
x_dot = rocket.f(x,u)


%% QUESTIONS:
% what are normal values for P1 and P2?


