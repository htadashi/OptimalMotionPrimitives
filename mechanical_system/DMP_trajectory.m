function [TRJ, TT] = DMP_trajectory(DMP, dt, t_end, obs, l_1, l_2)
% DMP_TRAJECTORY Reconstruct DMP after changing its goal
    % initialize the DMP state
    S.x = 1; 
    S.y = DMP.y0; 
    S.z = DMP.dy0*DMP.tau;
    S.t = 0;
    t = 0;
    TRJ = [S.y]; TT = [t]; TRJD = [DMP.dy0];
    % reconstruct the trajectory by Euler integration (F times higher than
    % the trajectory sampling rate)
    F = 1;
    dt = dt / F;
    while t < t_end
        for i = 1:F
            if DMP.type == 1
                S = DMP_integrate(DMP, S, dt);
            elseif DMP.type == 2
                S = OMF_integrate(DMP, S, dt);
            elseif DMP.type == 3 && obs ==1
                obs_pos = obstacle_position();
                S = DMP_integrate_with_obstacles(DMP, S, dt, obs_pos, l_1, l_2);
            end
            t = t + dt;
        end
        TT = [TT; t];
        TRJ = [TRJ; S.y];
    end

%% Another idea about obstacle avoidance 
%{
    if obs == 1
        for i = 1:size(TRJ, 1)
            if TRJ(i,1)<0
                TRJ(i,1)=0;
            elseif TRJ(i,1)>pi
                TRJ(i,1)=pi;
            end
            if TRJ(i,2)>pi
                TRJ(i,2) = pi;
            elseif TRJ(i,2)<-pi
                TRJ(i,2) = -pi;
            end
            if 2*TRJ(i,1)+TRJ(i,2)>2*pi
                TRJ(:,2) = 2*pi -2*TRJ(i,1);
            elseif 2*TRJ(i,1)+TRJ(i, 2)<0
                TRJ(i,2) = TRJ(i,1);
            end
        end
    end
%}
end