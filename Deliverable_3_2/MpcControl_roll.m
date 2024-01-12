classdef MpcControl_roll < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing
            
            [nx, nu] = size(mpc.B); %(2x1)
            
            % Steady-state targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1); % (2x1)
            u_ref = sdpvar(nu, 1);  % (1x1)
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N); % (2,N)
            U = sdpvar(nu, N-1); % (1,N-1)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE

            Hu = [1;-1]; hu = [20;20]; %Pdiff +-20% 

            % Tuning matrices
            Q=200*eye(nx); 
            R=0.2*eye(nu);

            % Compute terminal weight Qf
            [~,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);

        
            % set the problem constraints and objectives
            obj = 0;
            con = [];

            obj = (U(:,1)-u_ref)'*R*(U(:,1)-u_ref) + (X(:,1)-u_ref)'*Q*(X(:,1)-u_ref);
            con = ((X(:,2)-x_ref) == mpc.A*(X(:,1)-x_ref)+mpc.B*(U(:,1)-u_ref))+(Hu * U(:, 1) <= hu);
             
            for i = 2:N-1
                con = con + ((X(:,i+1)-x_ref )== mpc.A*(X(:,i)-x_ref) + mpc.B*(U(:,i)-u_ref));
                con = con + (Hu*U(:,i) <= hu);
                obj = obj + (X(:,i)-x_ref(:,1))'*Q*(X(:,i)-x_ref(:,1)) ...
                    + (U(:,i)-u_ref(:,1))'*R*(U(:,i)-u_ref(:,1));
            end
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref);
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Steady-state targets
            nx = size(mpc.A, 1);
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            
            Hu = [1;-1]; hu = [20;20]; %Pdiff +-20% 

            % Define constraints
            Q = eye(1); 
          
            con = (xs == mpc.A * xs + mpc.B * us);
            con = con + (Hu*us <= hu); 
            obj = (mpc.C*xs-ref)'*Q*(mpc.C*xs-ref);
            
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
