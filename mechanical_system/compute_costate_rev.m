function p = compute_costate_rev(x, u, M)
% COMPUTE_COSTATE_REV  Compute the co-state of the reverse system 
%                      x'(t) = -f(x(t))-g(x(t))u(t)

   dVdq_dot = zeros(size(u));
   for t = 1:size(u, 2) % samples
        dVdq_dot(t,:) = 2*M(x(t,:))*u(t,:)';
   end
   p = dVdq_dot;
   %p = p';
end