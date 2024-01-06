classdef MpcControl_y < MpcControlBase
    
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
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system

            % subsys 2: sys_y
            % states: x_y = (wx, alpha, vy, y)      input: u_y = (d1)

            % Input Constraints : 
            % u in U = { u | Mu <= m }
            M = [1;-1]; m = [0.26; 0.26];

            % Output constraints:
            % x in X = { x | Fx <= f }
            F = [1 0 0 0; 0 1 0 0; 
                0 0 1 0; 0 0 0 1;
                -1 0 0 0; 0 -1 0 0; 
                0 0 -1 0; 0 0 0 -1]; 
            f = [100; 0.1745; 100; 100; 100; 0.1745; 100; 100];
            
            % Tuning
            Q = eye(nx); 
            R = 150*eye(nu);

                      
            % LQR
            sys = LTISystem('A',mpc.A,'B',mpc.B);
            sys.x.min = [-100; -0.1745; -100; -100]; sys.x.max = [100; 0.1745; 100; 100];
            sys.u.min = [-0.26]; sys.u.max = [0.26];
            sys.x.penalty = QuadFunction(Q); sys.u.penalty = QuadFunction(R);
            
            Xf = sys.LQRSet;
            Qf = sys.LQRPenalty.H;
            [Ff,ff]=double(polytope(Xf));

            % Plot terminal sets
            figure;
            tiledlayout(1, 3);
            nexttile;
            Xf.projection(1:2).plot();
            title('terminal set wx and alpha');
            nexttile;
            Xf.projection(2:3).plot();
            title('terminal set alpha and vy');
            nexttile;
            Xf.projection(3:4).plot();            
            title('terminal set vy and y');

            % Constraints and objectives
            obj = U(:,1)'*R*U(:,1) + X(:,1)'*Q*X(:,1);
            con = (X(:,2) == mpc.A*X(:,1) + mpc.B*U(:,1)) ...
                + (M*U(:,1) <= m) + (F*X(:,1) <= f);

            for i = 2:N-1
                con = con + (X(:,i+1) == mpc.A*X(:,i) + mpc.B*U(:,i));
                con = con + (F*X(:,i) <= f) + (M*U(:,i) <= m); 
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
            
            nx = size(mpc.A, 1);

            % Steady-state targets
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
