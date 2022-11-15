%% Script to plot Figures 2 and 3 of paper 
%% "Learning optimal controllers: A dynamical motion primitive approach"

% Sampling grid 
clear all
load 'data/sampling_plots.mat'

%% Set figure properties.
set(0,'defaultFigureColor','white');
%% Set grids on all axis.
set(0,'defaultAxesXGrid','on');
set(0,'defaultAxesYGrid','on');

m_samples = cell2mat(samples);

% Compare cost of optimal solution with cost of DMP solution
subplot(4,1,1:2)
plot(x_values(1,:), optimal_costs, ...
     x_values(1,:), DMP_costs, ...
     x_values(1,:), estimated_costs, 'g-.');

xline(m_samples(1,:),'-.');
legend('Optimal cost (ground truth)', 'Cost of DMP solution', 'Estimated value for optimal cost', 'Location', 'northwest');

ylabel('Cost');
xlim([1,max(m_samples(1,:))])

% Estimation error
subplot(4,1,3)
plot(x_values(1,:), estimated_costs - optimal_costs);
xline(m_samples(1,:),'-.');
legend('Estimate error ', 'Location', 'northwest');
ylabel('Cost error');
xlim([1,max(m_samples(1,:))])

% DMP error
subplot(4,1,4)
plot(x_values(1,:), DMP_costs - optimal_costs);
xline(m_samples(1,:),'-.');
legend('DMP cost error ', 'Location', 'northwest');
ylabel('Cost error');
xlabel('x_1(t_f)');
xlim([1,max(m_samples(1,:))])

% Sampling grid 
figure
subplot(2,1,1);
scatter(m_samples(1,:), m_samples(2,:));
% title('Sampling grid');
xlabel('x_1(t_f)');
ylabel('x_2(t_f)');
xlim([1,max(m_samples(1,:))])
title('Proposed method')

% Metrics
max_error = max(abs(optimal_costs - estimated_costs))
max_relative_error = max(abs(optimal_costs - estimated_costs)./optimal_costs) * 100
min_diff_sample = min(diff(m_samples(1,:)))
total_samples_uniform = ceil((max(m_samples(1,:)) - 1) / min_diff_sample)
samples_used = size(m_samples, 2)-1
increase_percentual = (total_samples_uniform/samples_used - 1)*100

subplot(2,1,2);
uniform_samples = m_samples(1,1):min_diff_sample:m_samples(1,end);
scatter(uniform_samples, ones(1, length(uniform_samples)))
xlim([1,max(m_samples(1,:))])
xlabel('x_1(t_f)');
ylabel('x_2(t_f)');
title('Uniform sampling')   