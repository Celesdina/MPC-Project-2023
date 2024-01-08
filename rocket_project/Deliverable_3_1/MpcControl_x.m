classdef MpcControl_x < MpcControlBase
    
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

            % subsys 1: sys_x 
            % states: x_x = (wy, beta, vx, x)       input: u_x = (d2)
            % Constraints : 
            % u in U = { u | Mu <= m }
            M = [1;-1]; m = [0.26; 0.26];
            % x in X = { x | Fx <= f }
            F = [1 0 0 0; 0 1 0 0; 
                0 0 1 0; 0 0 0 1;
                -1 0 0 0; 0 -1 0 0; 
                0 0 -1 0; 0 0 0 -1]; 
            f = [100; 0.1745; 100; 100; 100; 0.1745; 100; 100];
            
            Q = 1 * eye(4);
            R = 1;
            
            % Compute LQR controller for unconstrained system
            [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
            % MATLAB defines K as -K, so invert its signal
            K = -K; 
            
            % Compute maximal invariant set
            Xf = polytope([F;M*K],[f;m]);
            Acl = [mpc.A+mpc.B*K];
            while 1
                prevXf = Xf;
                [T,t] = double(Xf);
                preXf = polytope(T*Acl,t);
                Xf = intersect(Xf, preXf);
                if isequal(prevXf, Xf)
                    break
                end
            end
            [Ff,ff] = double(Xf);
            
            % % MPT version
            sys = LTISystem('A',mpc.A,'B',mpc.B);
            sys.x.min = [-100; -0.1745; -100; -100]; sys.x.max = [100; 0.1745; 100; 100];
            sys.u.min = [-0.26]; sys.u.max = [0.26];
            sys.x.penalty = QuadFunction(Q); sys.u.penalty = QuadFunction(R);

            Xf_mpt = sys.LQRSet;
            Qf_mpt = sys.LQRPenalty;
            
            tiledlayout(1, 3);
            nexttile;
            Xf_mpt.projection(1:2).plot();
            title('terminal set wy and beta');
            nexttile;
            Xf_mpt.projection(2:3).plot();
            title('terminal set beta and vx');
            nexttile;
            Xf_mpt.projection(3:4).plot();            
            title('terminal set vx and x');

            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = (X(:,2) == mpc.A*X(:,1) + mpc.B*U(:,1)) + (M*U(:,1) <= m);

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