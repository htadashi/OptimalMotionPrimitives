%[casadi_path, omf_path, dmp_path] = load_paths();

% save as a gif
gif_filename = 'opt_2dof_pendulum_example.gif';
delete(gif_filename);

theta_1 = pi/4 * ones(1,100)';  % θ₁
theta_2 = linspace(0,2*pi,100)'; % θ₂

%plot(theta_1, theta_2)
anim_inc = 1;

%% Second link constant, first moving 
q= [theta_2(1:anim_inc:end)';  % θ₁
    theta_1(1:anim_inc:end)']; % θ₂

%% First link constant, second moving 
%q=[theta_1(1:anim_inc:end)';  % θ₁
%   theta_2(1:anim_inc:end)']; % θ₂

%plot_pendulum(scenario, q_DMP, gif_filename)


% decrease size of trajectory by 10 in order to run faster

scenario = 1;

plot_pendulum(scenario, q, gif_filename)