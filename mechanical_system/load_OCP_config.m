function [n_interval, t_final, scenario] = load_OCP_config(experiment_number)

    switch experiment_number
        case 1
            n_interval = 100; % number of control intervals
            t_final = 5;
            scenario = 1;
        % define other cases here
        otherwise
            warning("Invalid experiment number")
    end

end