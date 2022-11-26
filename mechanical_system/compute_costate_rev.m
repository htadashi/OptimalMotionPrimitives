function p = compute_costate_rev(x, u, g)
% COMPUTE_COSTATE_REV  Compute the co-state of the reverse system 
%                      x'(t) = -f(x(t))-g(x(t))u(t)
   dVdX = zeros(size(u));
   for t = 1:size(x, 1) % number of samples
        dVdX(t,:) = 2*inv(g(x(t,:)))'*u(t,:)';
   end
   p = dVdX;
   %p = p';
end