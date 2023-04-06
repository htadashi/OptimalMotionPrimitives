function p = DMP_obstacle_avoidance_term(DMP, S, obs_pos, l_1, l_2)
% DMP_OBSTACLE_AVOIDANCE_TERM 
% This function is implemented based on the
% obstacle avoidance term in the paper
% "Biologically-inspired dynamical systems for movement generation: 
% automatic real-time goal adaptation and obstacle avoidance"

    if ~isfield(DMP, 'beta_obs')
        DMP.beta_obs = 20/pi;
    end

    if ~isfield(DMP, 'gama_obs')
        DMP.gama_obs = 10000;
    end

    % Preallocation
    % p = zeros(1,size(DMP.w,2));


    % Cartesian coordinates for the second link, end point
    x_2 =l_1*cos(S.y(1)) + l_2*cos(S.y(1)+S.y(2));
    y_2 =l_1*sin(S.y(1)) + l_2*sin(S.y(1)+S.y(2));
    xy_coords_2 = [x_2; y_2];

    % Velocity from the angular velocities
    v_1 = -(l_1*sin(S.y(1))+l_2*sin(S.y(1)+S.y(2)))*S.y(3) - l_2*sin(S.y(1)+S.y(2))*S.y(4);
    v_2 =  (l_1*cos(S.y(1))+l_2*cos(S.y(1)+S.y(2)))*S.y(3) + l_2*cos(S.y(1)+S.y(2))*S.y(4);    
    v = [v_1; v_2];
    
    eps = 1e-5;
    % Calculate the current angle between the obstacle and the 
    obs_angle = real(acos(dot(obs_pos-xy_coords_2,v))/(dot(abs(obs_pos - xy_coords_2), abs(v))+eps));

    % Rotation matrix: TO BE CHECKED AGAIN
    R = [cos(obs_angle), -sin(obs_angle); sin(obs_angle), cos(obs_angle)];

    p = DMP.gama_obs * obs_angle * exp(-DMP.beta_obs .* obs_angle) * R*v ;
    p = [p; 0; 0];
end

