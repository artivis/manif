clc; clear; 
%close all;

file = "se2_slerp.csv";
%file = "se2_cubic.csv";

data = load(file);

r = 0.1; % magnitude (length) of arrow to plot

x = data(:,1);
y = data(:,2);
t = data(:,3);

u = r * cos(t); % convert polar (theta,r) to cartesian
v = r * sin(t);

figure;
h = quiver(x,y,u,v);
set(gca, 'XLim', [-0.5 3], 'YLim', [-0.5 2]);
set(get(gca, 'title'), 'string', 'SE2 Interpolation (CUBIC-like)');