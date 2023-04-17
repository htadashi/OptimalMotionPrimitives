function cost = compute_cost(u, time)
% COMPUTE_COST  Compute the cost ∫ |u|²(s) ds
    if(size(u,2) ~= 2)
        disp('Warning: check if u argument is being passed right')
    end
    cost = trapz(time, vecnorm(u,2,2).^2);
end