function [t,u,x] = solve_OCP_direct_method(F, n_states, n_inputs, N, time_range, x0, x1)
% SOLVE_OCP_DIRECT_METHOD Solve OCP by direct method using RK4 integration
    opti = casadi.Opti(); % Object representing optimization problem

    % Solver config
    solver_options= struct;
    solver_options.ipopt.print_level = 0;
    solver_options.print_time =0;
    solver_options.verbose = 0;
    
    % ---- decision variables ---------
    X = opti.variable(n_states,N+1); % state trajectory
    U = opti.variable(n_inputs,N);   % control trajectory 
    ti = time_range(1);       
    tf = time_range(2);       
    T = tf - ti;              % time horizon 
    
    % ---- objective          ---------
    opti.minimize(dot(U,U)); 
        
    dt = T/N; % length of a control interval
    for k=1:N % loop over control intervals
        % Runge-Kutta 4 integration
        k1 = F(X(:,k),         U(:,k));
        k2 = F(X(:,k)+dt/2*k1, U(:,k));
        k3 = F(X(:,k)+dt/2*k2, U(:,k));
        k4 = F(X(:,k)+dt*k3,   U(:,k));
        x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
        opti.subject_to(X(:,k+1)==x_next); % close the gaps
    end
    
    % ---- boundary conditions --------
    opti.subject_to(X(:, 1)==x0);
    opti.subject_to(X(:, end)==x1);     

    % ---- constraints conditions for the double pendulum --------
    if n_states>=2 
        opti.subject_to(0<=X(1, :));
        opti.subject_to(X(1, :)<=pi);           
        opti.subject_to(-pi<=X(2, :));
        opti.subject_to(X(2, :)<=pi);
        opti.subject_to(2*X(1,:)+X(2, :)<=2*pi);
        opti.subject_to(2*X(1,:)+X(2, :)>=0);
    end

    %% Initial values
    opti.set_initial(X, zeros(n_states,N+1));
    opti.set_initial(U, ones(n_inputs,N));
    
    % ---- solve NLP              ------
    opti.solver('ipopt'); % set numerical backend
    sol = opti.solve();   % actual solve    
    
    x = sol.value(X);
    u = sol.value(U);
    t = linspace(0,sol.value(T),N+1);    
end