clc; clear; #close all;

path = "/home/user/";
file = strcat(path, "se2_interp.csv");

data = load(file);

'num k points'
num_k_pts = data(1,1)

'Interpolation method'
interp_method = data(1,2)

switch interp_method
  case 0
    method = 'SLERP'
  case 1
    method = 'CUBIC'
  case 2
    method = 'CN smooth'
  otherwise
    method = 'Unknown'
end

title = strcat('SE2 Interpolation (', method, ')');

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
set(get(gca, 'title'), 'string', title);
hold off

return;