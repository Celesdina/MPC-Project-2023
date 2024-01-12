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

            % Input constraints : 
            % u in U = { u | Mu <= m }
            M = [1;-1]; m = [0.26; 0.26];

            % State constraints:
            % x in X = { x | Fx <= f }
            F = [1 0 0 0; 0 1 0 0; 
                0 0 1 0; 0 0 0 1;
                -1 0 0 0; 0 -1 0 0; 
                0 0 -1 0; 0 0 0 -1]; 
            f = [100; 0.1745; 100; 100; 100; 0.1745; 100; 100];
            
            % Tuning
            Q = diag([100 1 1 100]);
            R = 0.1;


            % Compute terminal weight Qf
            [~,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);


            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];


            obj = (U(:,1)-u_ref)'*R*(U(:,1)-u_ref) + (X(:,1)-u_ref)'*Q*(X(:,1)-u_ref);
            con = (X(:,2)-x_ref == mpc.A*(X(:,1)-x_ref) + mpc.B*(U(:,1)-u_ref));
            con = con + (F*(X(:,1)-x_ref) <= f) + (M*(U(:,1)-u_ref) <= m);

            for i = 2:N-1
                con = con + ((X(:,i+1)-x_ref) == mpc.A*(X(:,i)-x_ref) + mpc.B*(U(:,i)-u_ref));
                con = con + (F*(X(:,i)-x_ref) <= f) + (M*(U(:,i)-u_ref) <= m); 
                obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref) + (U(:,i))'*R*(U(:,i)-u_ref);
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
            
            Q = eye(1); 
            
            H = [1;-1];
            h = [0.26; 0.26];

            % Constraints and objectives
            con = (xs == mpc.A*xs + mpc.B*us);
            con = con + (H*us <= h);
            obj = (mpc.C*xs-ref)'*Q*(mpc.C*xs-ref);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
