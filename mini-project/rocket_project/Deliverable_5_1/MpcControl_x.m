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
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            epsi = sdpvar(nx, N-1);
            A = mpc.A;
            B = mpc.B;
            sys = LTISystem('A',A,'B',B);
            Q = diag([25, 8, 10,  50]);
            R = eye(nu)*0.001;
            us = 0;
            umax = deg2rad(15) - us;
            umin = -umax - us;
            xmax = [inf; deg2rad(7); inf; inf];
            xmin = -xmax;
            sys.x.penalty = QuadFunction(Q); 
            sys.u.penalty = QuadFunction(R);
            sys.x.min = xmin; 
            sys.x.max = xmax;
            sys.u.min = umin; 
            sys.u.max = umax;
%             Xf = sys.LQRSet;
            Qf = sys.LQRPenalty.weight;
            % terminal set and cost
            sys.x.with('terminalPenalty');
            sys.x.terminalPenalty = QuadFunction(Qf);
%             sys.x.with('terminalSet');
%             sys.x.terminalSet = Xf;
            % Constraints
            % u in U = { u | Mu <= m }
            M = [eye(nu);-eye(nu)]; m = [umax; umax];
            % x in X = { x | Fx <= f }
            F = [eye(nx); -eye(nx)]; f = [xmax;xmax];
            S = eye(nx)*5;
            con = (X(:,2)-x_ref == A*(X(:,1)-x_ref) + B*(U(:,1)-u_ref)) + (M*U(:,1) <= m);
            obj = (U(:,1)-u_ref)'*R*(U(:,1)-u_ref);
            for i = 2:N-1
%                 F = [eye(nx); -eye(nx)]; f = [xmax;xmax]+[epsi(:,i);epsi(:,i)];
                con = con + (X(:,i+1)-x_ref == A*(X(:,i)-x_ref) + B*(U(:,i)-u_ref));
                con = con + (F*X(:,i) <= f) + (M*U(:,i) <= m);
%                 obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref) + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref) + epsi(:,i)'*S*epsi(:,i)+5*norm(epsi(:,i), 1);
                obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref) + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref);
            end
%             con = con + (Xf.A*X(:,N) <= Xf.b);
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref);
%             %Plot terminal set
%             figure;
%             Xf.projection(1:2).plot();
%             title('Terminal set of Controller X projected onto (1,2)')
%             figure;
%             Xf.projection(2:3).plot();
%             title('Terminal set of Controller X projected onto (2,3)')
%             figure;
%             Xf.projection(3:4).plot();
%             title('Terminal set of Controller X projected onto (3,4)')
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
%             ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
%                 {X(:,1), x_ref, u_ref}, {U(:,1), X, U});

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
            A = mpc.A; B = mpc.B; C = mpc.C; D = mpc.D;
            umax = deg2rad(15);
            xmax = [inf; deg2rad(7); inf; inf];
            F = [eye(nx); -eye(nx)];
            f = [xmax; xmax];
            M = [1; -1];
            m = [umax; umax];
            con = [xs == A*xs + B*us, ref == C*xs + D*us, F*xs <= f, M*us <= m];
            obj = us'*us;           
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
