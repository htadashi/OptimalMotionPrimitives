% This script solves the optimal control problem for maximizing the
% link-velocity of a 1 DoF Pendulum actuated by a BSA.

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

%% Optimization
disp('The optimisation problem is getting created.');

% Nodes
N = 200;

% Initial state
x0 = zeros(4,1);

% x_initial = [0; 0; 0; 0];

%q0_deg = -90;

%x0(1) =  deg2rad(q0_deg);
%x0(3) =  deg2rad(q0_deg);
%x0(5) =  deg2rad(q0_deg);

% Final time
T_final = 1;

% Cost function (to be evaluated at the EE)
cost = @(u) vecnorm(u').^2;

%[t_sol,u_sol,x_sol,dLambda_t_sol] = opt_2dof_bsa(N, T_final, x0, cost);
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

%% Display link and motor speeds
figure
subplot(2,1,1);
plot(time_gt,theta_dot_gt(1,:),'DisplayName','theta dot gt 1')
hold on
plot(t_sol,theta_dot_sol(1,:),'DisplayName','theta dot sol 1')
%color_area_tol(t_sol, dLambda_t_sol(1,:), get(gca, 'YLim'), 1e-1, [0 0 1], 'dLambda_t1');
%color_area_tol(t_sol, dLambda_t_sol(3,:), get(gca, 'YLim'), 1e-1, [1.0000 0.3765  0.1098], 'dLambda_t3');
legend('Location','southwest')

subplot(2,1,2); 
plot(time_gt,theta_dot_gt(2,:),'DisplayName','theta dot gt 2')
hold on
plot(t_sol,theta_dot_sol(2,:),'DisplayName','theta dot sol 2')
%color_area_tol(t_sol, dLambda_t_sol(2,:), get(gca, 'YLim'), 1e-1, [0 0 1], 'dLambda_t2');
%color_area_tol(t_sol, dLambda_t_sol(4,:), get(gca, 'YLim'), 1e-1, [1.0000 0.3765  0.1098], 'dLambda_t4');
legend('Location','southwest')

%% Display EE speed and objective value
%{
vEE = {};
obj = {};

for i=1:size(theta_dot_sol(1,:),1)
    v = get_vEE(x_sol(:,i));
    vEE{end+1} = v;
    obj{end+1} = cost(v);
end

vEE = [vEE{:}];
obj = [obj{:}];

figure 
subplot(2,1,1);
plot(t_sol, obj,'DisplayName','obj')
legend

subplot(2,1,2);
hold on
plot(t_sol, vEE(1,:),'DisplayName','vEE(1)')
plot(t_sol, vEE(2,:),'DisplayName','vEE(2)')
plot(t_sol, vEE(3,:),'DisplayName','vEE(3)')
legend

% for i=1:size():

%% Display link, motor and spring positions

figure
subplot(2,1,1);
plot(t_sol,q_sol(1,:),'DisplayName','q(1)')
hold on
plot(t_sol,theta_sol(1,:),'DisplayName','theta(1)')
plot(t_sol,psi_sol(1,:),'--','DisplayName','psi(1)')
color_area_tol(t_sol, dLambda_t_sol(1,:), get(gca, 'YLim'), 1e-1, [0 0 1], 'dLambda_t1');
color_area_tol(t_sol, dLambda_t_sol(3,:), get(gca, 'YLim'), 1e-1, [1.0000 0.3765  0.1098], 'dLambda_t3');
legend('Location','southwest')

subplot(2,1,2); 
plot(t_sol,q_sol(2,:),'DisplayName','q(2)')
hold on
plot(t_sol,theta_sol(2,:),'DisplayName','theta(2)')
plot(t_sol,psi_sol(2,:),'--','DisplayName','psi(2)')
color_area_tol(t_sol, dLambda_t_sol(2,:), get(gca, 'YLim'), 1e-1, [0 0 1], 'dLambda_t2');
color_area_tol(t_sol, dLambda_t_sol(4,:), get(gca, 'YLim'), 1e-1, [1.0000 0.3765  0.1098], 'dLambda_t4');
legend('Location','southwest')

%% Display clutches
figure
subplot(2,1,1);
plot(t_sol(1:end-1), dLambda_t_sol(1,:),'DisplayName','dLambda_t1')
hold on
plot(t_sol(1:end-1), dLambda_t_sol(3,:),'DisplayName','dLambda_t3')
legend

subplot(2,1,2);
plot(t_sol(1:end-1), dLambda_t_sol(2,:),'DisplayName','dLambda_t2')
hold on
plot(t_sol(1:end-1), dLambda_t_sol(4,:),'DisplayName','dLambda_t4')
legend
hold off
%}
%% Visualize the solution
plot_pendulum(u_sol', filename);
% figure

% plot(t_sol,dpsi_sol(1) - dq_sol(1))
% 
% hold on
% plot(t_sol,dpsi_sol(1))
%% Helper functions
function color_area_tol(x, indicator, y_lim, tol, color, name)
    y_max = y_lim(2); 
    y_min = y_lim(1);
    positive = abs(indicator)>tol;
    k = find(diff(positive)); 
    if positive(1)>0
        k = [1 k];
    end
    if positive(end)>0
        k = [k size(positive,2)];
    end
    
    x_points = x(repelem(k,2));
    y_points = repmat([y_min y_max y_max y_min],1,size(k,2)/2);
    
    a = fill(x_points, y_points, color,'DisplayName',name);
    if ~isempty(a)
        a.FaceAlpha = 0.05;
        a.EdgeAlpha = 0;
    end
end
