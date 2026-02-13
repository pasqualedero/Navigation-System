classdef EkfSlam < handle
    % EkfSlam: Handles EKF-SLAM for an Ackermann vehicle
    % Features: Dynamic state expansion, Active Batch Update, Ackermann Kinematics
    
    properties
        mu      % State Vector [x; y; theta; L1x; L1y; ... Lnx; Lny]
        Sigma   % Covariance Matrix
        Q       % Process Noise (Robot motion)
        R       % Measurement Noise (Single landmark)
    end
    
    methods
        function obj = EkfSlam(initialPose, numLandmarks, P, Q_robot, R_sensor)
            % Initialize State: Robot Pose + NaNs for Landmarks
            obj.mu = [initialPose(:); NaN(2 * numLandmarks, 1)];
            
            % Store parameters
            obj.Sigma = P;
            obj.Q = Q_robot; 
            obj.R = R_sensor;
        end

        function predict(obj, u, Ts)

            xr = obj.mu(1); yr = obj.mu(2); th = obj.mu(3);
            vx = u(1); vy = u(2); w = u(3);

            dx = cos(th)*vx - sin(th)*vy; 
            dy = sin(th)*vx + cos(th)*vy; 
            dth = w;

            xr = xr+dx*Ts; 
            yr = yr+dy*Ts; 
            th = th+dth*Ts;

            obj.mu(1:3) = [xr;yr;th];

            dim = length(obj.mu);

            F = eye(dim);

            F(1,3)=(-sin(th)*vx - cos(th)*vy)*Ts;
            F(2,3)=( cos(th)*vx - sin(th)*vy)*Ts;

            %Q_full = blkdiag(obj.Q, zeros(dim-3));

            obj.Sigma = F * obj.Sigma * F' + obj.Q;
        end

        function correct(obj, observations)
            % CORRECT STEP: Active Batch Update
            % observations: [ID, Range, Bearing; ...]
            
            if isempty(observations)
                return; % No landmarks seen, skip update
            end
            
            % Helper for angle wrapping
            wrapToPi = @(a) atan2(sin(a), cos(a));
            
            % 1. Initialization Loop
            % Before we update, ensure all seen landmarks exist in the state
            for i = 1:size(observations, 1)
                id = observations(i, 1);
                r  = observations(i, 2);
                b  = observations(i, 3);
                
                idx = 3 + (2*id - 1) : 3 + (2*id);
                
                % If landmark is NaN (new), initialize it
                if isnan(obj.mu(idx(1)))
                    th = obj.mu(3);
                    obj.mu(idx) = obj.mu(1:2) + r * [cos(b + th); sin(b + th)];
                end
            end
            
            % 2. Build Batch Matrices (Only for ACTIVE observations)
            N_obs = size(observations, 1);
            dim = length(obj.mu);
            
            H_batch = zeros(2 * N_obs, dim);  % Stacked Jacobian
            y_batch = zeros(2 * N_obs, 1);    % Stacked Residual
            
            for i = 1:N_obs
                id = observations(i, 1);
                r_meas = observations(i, 2);
                b_meas = observations(i, 3);
                
                idx = 3 + (2*id - 1) : 3 + (2*id);
                
                % Calculate Expected Measurement
                dx = obj.mu(idx(1)) - obj.mu(1);
                dy = obj.mu(idx(2)) - obj.mu(2);
                q = dx^2 + dy^2;
                r_pred = sqrt(q);
                b_pred = atan2(dy, dx) - obj.mu(3);
                
                % Fill Residual (Innovation)
                row_idx = (2*i - 1) : (2*i);
                y_batch(row_idx) = [r_meas - r_pred; 
                                    wrapToPi(b_meas - b_pred)];
                
                % Fill Jacobian H
                % Robot Part
                H_batch(row_idx, 1:3) = [-dx/r_pred, -dy/r_pred, 0; 
                                          dy/q,      -dx/q,     -1];
                % Landmark Part
                H_batch(row_idx, idx) = [ dx/r_pred,  dy/r_pred; 
                                         -dy/q,       dx/q];
            end
            
            % 3. Single Batch Update
            % Create block-diagonal R matrix for the active sensors
            R_batch = kron(eye(N_obs), obj.R);
            
            % Calculate Kalman Gain
            % Inversion size is (2*N_obs)x(2*N_obs). E.g., 4x4 or 6x6. FAST.
            S = H_batch * obj.Sigma * H_batch' + R_batch;
            K = obj.Sigma * H_batch' / S;
            
            % Update State and Covariance
            obj.mu = obj.mu + K * y_batch;
            obj.Sigma = (eye(dim) - K * H_batch) * obj.Sigma;
        end
        
        function pose = getPose(obj)
            pose = obj.mu(1:3);
        end
        
        function map = getMap(obj)
             % Returns [x, y] of all landmarks
             map = reshape(obj.mu(4:end), 2, [])';
        end
    end
end