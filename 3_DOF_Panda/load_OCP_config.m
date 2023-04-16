function [n_interval, t_final, scenario] = load_OCP_config(experiment_number)

    switch experiment_number
        case 1
            n_interval = 100; % number of control intervals
            t_final = 3;
            scenario = 1;
        case 2
            n_interval = 100; % number of control intervals
            t_final = 3;
            scenario = 1;
        case 3
            n_interval = 200; % number of control intervals
            t_final = 5;
            scenario = 1;
        case 4
            n_interval = 200; % number of control intervals
            t_final = 3;
            scenario = 1;  
        case 5
            n_interval = 300; % number of control intervals
            t_final = 1;
            scenario = 1;
        case 6  % with the  constraints, the solver does not solve
            n_interval = 120; % number of control intervals
            t_final = 2;
            scenario = 1;
        case 7  % with the  constraints, the solver does not solve
            n_interval = 200; % number of control intervals
            t_final = 2;
            scenario = 1;
        case 8
            n_interval = 200; % number of control intervals
            t_final = 1;
            scenario = 1;
        case 9
            n_interval = 200; % number of control intervals
            t_final = 1;
            scenario = 1;
        case 10
            n_interval = 100; % number of control intervals
            t_final = 3;
            scenario = 2;
        case 11
            n_interval = 300; % number of control intervals
            t_final = 3;
            scenario = 1;
        otherwise
            warning("Invalid experiment number")
    end

end