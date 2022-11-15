function DMP = OMF_train(y, time, DMP)
% Encodes the input trajectory y sampled at time with a discrete DMP.
% First, the derivatives of the provided trajectory are calculated. Then,
% the DMP structure is defined and the parameters of the basis functions 
% are calculated. The weights w are then calculated from the input trajectory
% in the least square sense.
%
% INPUTS:
%   y: the input trajectory. Contains position values of all the DOFs.
%   time: times at which the trajectory is sampled
%   DMP.N: number of basis functions to encode the trajectory. Optional, default = 10.
%   DMP.a_z: constant alpha of the dynamical system. Optional, default = 48.
%   DMP.a_x: constant alpha of the canonical system. Optional, default = 2.
% 
% OUTPUT:
%   DMP: the DMP structure containing all the calculated parameters.

%% check parameters, some are optional
if ~exist('DMP')
  DMP.N = 25;
elseif ~isfield(DMP, 'N')
  DMP.N = 25; 	% number of basis functions to encode the path
end
if ~isfield(DMP, 'D')
  DMP.D = 48;	% damping parameter
end
% time-varying stiffness parameters
if ~isfield(DMP, 'g_k')
  DMP.g_k = DMP.D / 4;  % max. stiffness
end
if ~isfield(DMP, 'k_0')
  DMP.k_0 = DMP.D / 2; 
end
if ~isfield(DMP, 'k_k')
  DMP.k_k = DMP.D;
end
if ~isfield(DMP, 'a_x')
  DMP.a_x = 2;
end
if ~isfield(DMP, 'is_zero')
  DMP.is_zero = 1;
end

%% compute derivatives
[NT, NS] = size(y);
if length(time) == 1
  time = linspace(0, (NT-1)*time, NT);
end
time = time - time(1);
for dof = 1:NS
  dy_dt(:,dof) = gradient(y(:,dof), time);
  ddy_dt(:,dof) = gradient(dy_dt(:,dof), time);
end

%% define the rest of the DMP parameters
DMP.tau = time(end);    % DMP time constant
tau = DMP.tau;
DMP.goal  = y(end,:);   % goal - final value
DMP.y0 = y(1,:);        % initial position
DMP.dy0 = dy_dt(1,:);   % initial velocity

% define Gausian kernel functions
c_lin = linspace(0, 1, DMP.N);
DMP.c = exp(-DMP.a_x * c_lin);	      % centers of gaussians
DMP.sigma2 = (diff(DMP.c)*0.75).^2;	  % widths of gaussians
DMP.sigma2 = [DMP.sigma2, DMP.sigma2(end)];

%% compute weights by linear regression
ft = zeros(NT, NS);
A = zeros(NT, DMP.N);
% progression of phase
x = exp(-DMP.a_x * time / DMP.tau);
% time-varying stifness
kappa = zeros(NT, NS);
for dof = 1:NS
    for i = 1:length(time)
        kappa(i,dof) = DMP.g_k * (1 / (1 + exp(-DMP.k_k*time(i))*(DMP.g_k/DMP.k_0 - 1)));
    end 
end
% target for fitting
for dof = 1:NS
  ft(:,dof) = -tau^2*ddy_dt(:,dof) + kappa(:,dof).*(DMP.goal(dof) - y(:,dof)) - DMP.D*DMP.tau*dy_dt(:,dof);  
end

for k = 1:NT
  % values of kernel functions at current phase
  psi = exp(-0.5 * (x(k) - DMP.c).^2 ./ DMP.sigma2)';
  % one row of regression matrix
  A(k,:) = x(k) * psi / sum(psi);
end

DMP.w = A\ft;

% LSQLIN can be used to guarantee that f is equal to zero at the end of movement. 
% for i = 1:NS
%   DMP.w(:,i) = lsqlin(A(2:NT-1,:), ft(2:NT-1,i), [], [], [A(1,:) ; A(NT,:)], [ft(1, i); 0]);
% end