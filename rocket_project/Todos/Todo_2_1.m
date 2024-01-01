addpath(fullfile('..', 'src'));

rocket = Rocket(Ts);

[xs,us] = rocket.trim()
sys = rocket.linearize(xs,us)