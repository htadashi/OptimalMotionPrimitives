function [x_initial, x_direction, x_final, N_samples, x_step, max_step, threshold] = load_OMF_config(experiment_number)
   
    switch experiment_number
        case 1
            x_initial = [0; 0; 0; 0];
            x_direction = [0.1; 0; 0.1; 0.1];
            x_final = [pi/2; 0; 0; 0];  
            N_samples = 8;
            x_step = 0.05; % Spatial increment
            max_step = 5; % Number of steps before forcing taking another sample
            threshold = 5;
        case 2
            x_initial = [0; 0; 0; 0];
            x_direction = [-0.1; 0.2; 0.5; 0.5];
            x_final = [-pi; 2*pi; 0; 0];  
            N_samples = 25;
            x_step = 0.2; % Spatial increment
            max_step = 5; % Number of steps before forcing taking another sample
            threshold = 5;
        case 3
            x_initial = [0; 0; 0; 0];
            x_direction = [0; 1; 0.1; 0.1];
            x_final = [0; 5; 0; 0];  
            N_samples = 30;
            x_step = 0.1; % Spatial increment
            max_step = 5; % Number of steps before forcing taking another sample
            threshold = 5;
        case 4
            x_initial = [pi; 0; 0; 0];
            x_direction = [-0.1; 0.1; 1; 1];
            x_final = [-pi; 2*pi; 0; 0];  
            N_samples = 30;
            x_step = 0.1; % Spatial increment
            max_step = 5; % Number of steps before forcing taking another sample
            threshold = 5;
        case 5
            x_initial = [0; 0; 0; 0];
            x_direction = [0.1; 0.1; 0.1; 0.1];
            x_final = [0; 0; 0; 0];  
            N_samples = 20;
            x_step = 0.05; % Spatial increment
            max_step = 5; % Number of steps before forcing taking another sample
            threshold = 5;
        case 6
            x_initial = [0; 0; 0; 0];
            x_direction = [1; 1; 1; 1];
            x_final = [0; 0; 0; 0];  
            N_samples = 8;
            x_step = 0.23; % Spatial increment
            max_step = 5; % Number of steps before forcing taking another sample
            threshold = 5;
        case 7
            x_initial = [0; 0; 0; 0];
            x_direction = [1; 1; 1; 1];
            x_final = [0.1; 0.1; 0; 0];  
            N_samples = 25;
            x_step = 0.27; % Spatial increment
            max_step = 5; % Number of steps before forcing taking another sample
            threshold = 5;
        case 8
            x_initial = [0; 0; 0; 0];
            x_direction = [1; 1; 1; 1];
            x_final = [0.5; 0.5; 0; 0];  
            N_samples = 20;
            x_step = 0.01; %0.025; % Spatial increment
            max_step = 5; % Number of steps before forcing taking another sample
            threshold = 5;
        case 9
            x_initial = [0; 0; 0; 0];
            x_direction = [1; 1; 1; 1];
            x_final = [0.5; 0.5; 0; 0];  
            N_samples = 10;
            x_step = 0.025; % Spatial increment
            max_step = 10; % Number of steps before forcing taking another sample
            threshold = 8;
        case 10
            x_initial = [0; 0; 0; 0];
            x_direction = [0.1; 0.1; 0.1; 0.1];
            x_final = [0; 0; 0; 0];  
            N_samples = 15;
            x_step = 0.1; % Spatial increment
            max_step = 5; % Number of steps before forcing taking another sample
            threshold = 5;
        case 11
            x_initial = [pi/2; pi/6; 0; 0];
            x_direction = [pi/2; pi/2; 0; 0];
            x_final = [0; 0; 0; 0];  
            N_samples = 20;
            x_step = 0.01; % Spatial increment
            max_step = 5; % Number of steps before forcing taking another sample
            threshold = 5;
        case 12
            x_initial = [pi/4; pi/4; 0; 0];
            x_direction = [0.15; 0.1; 0; 0];
            x_final = [3*pi/4; pi/2; 0; 0];  
            N_samples = 15;
            x_step = 0.1; % Spatial increment
            max_step = 7; % Number of steps before forcing taking another sample
            threshold = 7;
        otherwise
            warning("Invalid experiment number")
    end