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

            % u in U = { u | Mu <= m }
            M = [1;-1]; m = [0.26; 0.26];
            % x in X = { x | Fx <= f }
            F = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1; -1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 -1];
            f = [inf; 0.1222; inf; inf; inf; 0.1222; inf; inf];
            
            % Calculate the terminal set using MPT
            Q = N * eye(4);
            R = 1;
            sys = LTISystem('A', mpc.A, 'B', mpc.B);
            sys.x.min = -[inf; 0.1222; inf; inf]; sys.x.max = [inf; 0.1222; inf; inf];
            sys.u.min = -0.26; sys.u.max = 0.26;
            sys.x.penalty = QuadFunction(Q);
            sys.u.penalty = QuadFunction(R);
            
            F_mpt = sys.LQRSet;
            P_mpt = sys.LQRPenalty.weight;

            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = U(:,1)'* R *U(:,1);
            con = X(:,2) == mpc.A * X(:,1) + mpc.B * U(:,1);
            con = [con, M * U(:,1) <= m];

            for i = 2:N-1
                con = [con, (X(:,i+1) == mpc.A * X(:,i) + mpc.B * U(:,i))];
                con = [con, (F * X(:,i) <= f) + (M * U(:,i) <= m)];
                obj = obj + X(:,i)'*Q*X(:,i) + U(:,i)'*R*U(:,i);
            end
            con = [con, F_mpt.A*X(:,N) <= F_mpt.b];
            obj = obj + X(:,N)'*P_mpt*X(:,N);
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object  gurobi mosek
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
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj = 0;
            con = [xs == 0, us == 0];
            
%             A = mpc.A; B = mpc.B; C = mpc.C; D = mpc.D;
%             umax = deg2rad(15);
%             xmax = [inf; 0.0873; inf; inf];
%             F = [eye(nx); -eye(nx)];
%             f = [xmax; xmax];
%             M = [1; -1];
%             m = [umax; umax];
%             con = [xs == A*xs + B*us, ref == C*xs + D*us, F*xs <= f, M*us <= m];
%             obj = us'*us;   
            % Compute the steady-state target gurobi
            target_opti = optimizer(con, obj, sdpsettings('solver', 'mosek'), ref, {xs, us});
        end
    end
end
