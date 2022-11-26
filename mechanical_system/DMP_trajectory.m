function [TRJ, TT] = DMP_trajectory(DMP, dt, t_end)
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
            elseif DMP_type == 2
                S = OMF_integrate(DMP, S, dt);
            end
            t = t + dt;
        end
        TT = [TT; t];
        TRJ = [TRJ; S.y];
    end
end