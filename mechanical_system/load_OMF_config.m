function [x_initial, x_direction, x_final, N_samples, x_step, max_step, threshold] = load_OMF_config(experiment_number)
   
    switch experiment_number
        case 1
            x_initial = [0; 0; 0; 0];
            x_direction = [0.1; 0; 0.5; 0.5];
            x_final = [pi/2; 0; 0; 0];  
            N_samples = 25;
            x_step = 0.2; % Spatial increment
            max_step = 5; % Number of steps before forcing taking another sample
            threshold = 5;
        % define other cases here
        otherwise
            warning("Invalid experiment number")
    end