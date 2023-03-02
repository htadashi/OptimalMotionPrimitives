% This script plots an animated gif of the pendulum
clear
clc
close all

% Add path
casadi_path = "C:\Users\User\OneDrive\Desktop\Forschungspraktikum\CasADi\casadi-windows-matlabR2016a-v3.5.5";
addpath(genpath(casadi_path))
import casadi.*

% save as a gif
filename = 'opt_2dof_pendulum.gif';
delete opt_2dof_pendulum.gif

load('OMF_sampling_double_pendulum8_data.mat', 'time_training', 'u_training', 'x_training', 'time_gt', 'u_gt', 'x_gt'); 
t_sol = time_training;
u_sol = u_training;
x_sol = x_training;

% Extract states from solution

theta_sol  = x_sol(:, 1:2)';
theta_dot_sol = x_sol(:, 3:4)';

theta_gt = x_gt(1:2, :);
theta_dot_gt = x_gt(3:4,:);

% Extract inputs from solution
tau_sol = u_sol(1:2,:);

%% Display states
figure
subplot(2,1,1);
plot(time_gt,theta_dot_gt(1,:),'DisplayName','theta dot gt 1')
hold on
plot(t_sol,theta_dot_sol(1,:),'DisplayName','theta dot sol 1')
legend('Location','southwest')

subplot(2,1,2); 
plot(time_gt,theta_dot_gt(2,:),'DisplayName','theta dot gt 2')
hold on
plot(t_sol,theta_dot_sol(2,:),'DisplayName','theta dot sol 2')
legend('Location','southwest')

%% Visualize the solution
plot_pendulum(u_sol', filename);

