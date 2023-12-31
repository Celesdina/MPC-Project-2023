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
                        
            % State constraints 
            %Pdiff +-20% 
            Hu = [1;-1]; hu = [20;20]; 
            
            %LQR
            Q=200*eye(nx); 
            R=0.2*eye(nu);

            sys = LTISystem('A', mpc.A, 'B', mpc.B);
            sys.u.min = -20; sys.u.max= 20;
            sys.x.penalty= QuadFunction(Q); sys.u.penalty=QuadFunction(R);

            Xf=sys.LQRSet;
            Qf=sys.LQRPenalty.H;
            [Ff,ff]=double(polytope(Xf));
            
            % plot terminal invariant set
            figure
            hold on; grid on;
            plot(polytope(Xf),'r');
            title("terminal set: roll");
            
            % constraints and objectives
            obj = U(:,1)'*R*U(:,1) + X(:,1)'*Q*X(:,1);
            con = (X(:,2) == mpc.A*X(:,1)+mpc.B*U(:,1))+(Hu * U(:, 1) <= hu);
            for i = 2:N-1
                con = con + (X(:,i+1) == mpc.A*X(:,i) + mpc.B*U(:,i));
                con = con + (Hu*U(:,i) <= hu);
                obj = obj + X(:,i)'*Q*X(:,i) + U(:,i)'*R*U(:,i);
            end
            con = con + (Ff*X(:,N) <= ff);
            obj = obj + X(:,N)'*Qf*X(:,N);
            
            
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
            obj = 0;
            con = [xs == 0, us == 0];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
