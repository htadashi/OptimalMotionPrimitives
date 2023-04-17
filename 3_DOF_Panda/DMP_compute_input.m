function u_DMP = DMP_compute_input(x_DMP, time, u_fcn_x_dx)
% DMP_COMPUTE_INPUT  Compute input associated to the DMP trajectory.
%                    Assumes that u can be expressed as a function 
%                    of x and x', i.e. u = u_fcn_x_dx(x, dx) 
    n_states = size(x_DMP, 2);
    n_samples = length(time);

    u_DMP = {}; 
    % Compute x'_DMP using numerical derivative
    dx_DMP = [];
    for n=1:n_states
        dx_DMP = [dx_DMP gradient(x_DMP(:,n), time)];
    end

    % Recover control input from DMP
    for t=1:n_samples
        u_DMP{t} = u_fcn_x_dx(x_DMP(t,:)', dx_DMP(t,:)');
    end
    u_DMP = cell2mat(u_DMP);
end