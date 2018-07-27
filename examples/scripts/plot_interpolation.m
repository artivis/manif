clc; clear; 
%close all;

file = "/home/jeremie/workspaces/universe/build/manif/examples/se2_interp.csv";
%file = "se2_cubic.csv";


method = 'SLERP'
method = 'CUBIC-like'
method = 'C2 smooth'

title = strcat('SE2 Interpolation (', method, ')');

data = load(file);

'num k points'
num_k_pts = data(1,1)
'num interpolated points'
total_pts = size(data(2+num_k_pts:end,1), 1)

r = 0.1; % magnitude (length) of arrow to plot

x = data(2:end,1);
y = data(2:end,2);
t = data(2:end,3);

u = r * cos(t); % convert polar (theta,r) to cartesian
v = r * sin(t);

figure;
quiver(x(1:num_k_pts),y(1:num_k_pts),u(1:num_k_pts),v(1:num_k_pts),'color',[0 0 1]);
hold on
quiver(x(num_k_pts+1:end),y(num_k_pts+1:end),u(num_k_pts+1:end),v(num_k_pts+1:end),'color',[1 0 0]);
hold off
return;

figure;
h = quiver(x(1:num_k_pts),y(1:num_k_pts),u(1:num_k_pts),v(1:num_k_pts),'color',[0 0 1]);
set(gca, 'XLim', [-1.5 1.5], 'YLim', [-1 1]);
set(get(gca, 'title'), 'string', title);

quiver(x(num_k_pts+1:end),y(num_k_pts+1:end),u(num_k_pts+1:end),v(num_k_pts+1:end),'color',[1 0 0]);
set(gca, 'XLim', [-1.5 1.5], 'YLim', [-1 1]);
set(get(gca, 'title'), 'string', title);