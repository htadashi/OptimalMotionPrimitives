% Define constants needed for the double pendulum
function [m1, l1, m2, l2, Iz1, Iz2, r1, r2] = get_params_pendulum(scenario)
% For the computation of the moment of inertia, see first:
% https://en.wikipedia.org/wiki/Double_pendulum 
% https://en.wikipedia.org/wiki/List_of_moments_of_inertia
% Iz1 = (m1*l1*l1)/12;    % moment of inertia in z-direction of link 1
% Iz2 = (m2*l2*l2)/12;    % moment of inertia in z-direction of link 2

    if isempty(scenario)
        scenario = 1;
    end 

    switch scenario
        case 1        
            % Data for the lengths, centers of mass and masses
            % https://www.ijeert.org/papers/v6-i11/3.pdf
            % or:
            % https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9971718
            m1 = 1;
            l1 = 1;     
            m2 = 1;
            l2 = 1;       
            Iz1 = 0.5; 
            Iz2 = 0.5;
            % Assumption: center of mass of each limb is at its midpoint
            r1 = 0.5; 
            r2 = 0.5; 
        case 2
            % Data for the lengths, centers of mass and masses, see 
            % the document "OCD for swinging and damping
            % a double pendulum", page 115
            m1 = 38.4;   % kg
            l1 = 1.19;   % m
            m2 = 26;     % kg            
            l2 = 1;      % m
            Iz1 = 28.72; % kg*m^2
            Iz2 = 6.3;   % kg*m^2   
            r1 = 0.77;   % m
            r2 = 0.415;  % m
        otherwise
            warning("Invalid scenario")
    end

end