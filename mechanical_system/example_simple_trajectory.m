clc;
clear;

[casadi_path, omf_path, dmp_path] = load_paths();


% save as a gif
gif_filename = 'opt_2dof_pendulum_example.gif';
delete(gif_filename);


theta_1 = pi/4 * ones(1,100)';  % θ₁
theta_2 = linspace(0,2*pi,100)'; % θ₂

%plot(theta_1, theta_2)
anim_inc = 1;

%% Second link constant, first moving 
q1= [theta_2(1:anim_inc:end)';  % θ₁
    theta_1(1:anim_inc:end)']; % θ₂

%% First link constant, second moving 
q2=[theta_1(1:anim_inc:end)';  % θ₁
   theta_2(1:anim_inc:end)']; % θ₂

%% Plots
scenario = 1;


plot_pendulum(scenario, q1, gif_filename, 1)    % with constraints


%plot_pendulum(scenario, q1, gif_filename, 0)    % without constraints

%% From here on, everything is taken from the novel framework paper 

addpath('utils/');
import_gmp_lib();
import_io_lib();

%% Load training data
fid = FileIO('data_obstacles/pos_data.bin', FileIO.in);
Timed = fid.read('Timed');
Pd_data = fid.read('Pd_data');
dPd_data = fid.read('dPd_data');
ddPd_data = fid.read('ddPd_data');
fid.close();

Ts = Timed(2)-Timed(1);

%% initialize and train GMP
train_method = 'LS';
N_kernels = 30;
kernels_std_scaling = 1.5;
n_dof = size(Pd_data,1);
gmp = GMP(n_dof, N_kernels, kernels_std_scaling);
tic
offline_train_mse = gmp.train(train_method, Timed/Timed(end), Pd_data);
offline_train_mse
toc

gmp.setScaleMethod( TrajScale_Prop(n_dof) );

taud = Timed(end);
yd0 = gmp.getYd(0); %Pd_data(1);
ygd = gmp.getYd(1); %Pd_data(end);

kt = 1.5; % temporal scaling
ks = diag([1 1 1]); % spatial scaling
tau = taud/kt;
y0 = yd0 + 0;
% yg = ks*(ygd - yd0) + y0;
% yg = ygd + [0.1; -0.1; 0.23]; view_ = [171.5301, -2.3630];
yg = ygd + [0.7; -0.7; 0.05];  view_ = [171.9421, -3.0690];

%% ======== Limits ==========
%            lower limit     upper limit
pos_lim = [[-1.2 -1.2 0.22]' [1.2 1.2 0.5]'];
vel_lim = repmat([-0.3 0.3], 3,1);  % lower and upper limit, same for all DoFs
accel_lim = repmat([-0.4 0.4], 3,1);


%% ======== Generate trajectories ==========

qp_solver_type = 0; % matlab-quadprog:0 , osqp:1, Goldfarb-Idnani: 2

data = {};

%% --------- Demo -----------
data{length(data)+1} = ...
    struct('Time',Timed/kt, 'Pos',Pd_data, 'Vel',dPd_data*kt, 'Accel',ddPd_data*kt^2, 'linestyle','--', ...
    'color',[0.7 0.7 0.7], 'legend','demo', 'plot3D',true, 'plot2D',false);

%% --------- Proportional scaling -----------
gmp2 = gmp.deepCopy();
gmp2.setScaleMethod( TrajScale_Prop(n_dof) );
[Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp2, tau, y0, yg);
data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
    'color','blue', 'legend','DMP', 'plot3D',true, 'plot2D',true);

%% --------- Optimized DMP -> POS -----------
[Time, P_data, dP_data, ddP_data] = offlineGMPweightsOpt(gmp, tau, y0, yg, pos_lim, vel_lim, accel_lim, 1, 0, [], qp_solver_type); %getOptGMPTrajectory(gmp, tau, y0, yg, pos_lim, vel_lim, accel_lim, true, false);
data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
    'color',[0.72 0.27 1], 'legend','$DMP^*_p$', 'plot3D',true, 'plot2D',true);



