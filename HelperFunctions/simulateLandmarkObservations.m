function observations = simulateLandmarkObservations(truePose, landmarks, R)
    % Simulates sensor data: Returns [ID, Range, Bearing] for visible landmarks
    maxRange = 10; % Set your sensor range limit here
    observations = [];
    
    for i = 1:size(landmarks, 1)
        dx = landmarks(i, 1) - truePose(1); 
        dy = landmarks(i, 2) - truePose(2);
        
        % True Distance with Sensor Noise
        r = sqrt(dx^2 + dy^2) + normrnd(0,R(1,1));
        
        if r < maxRange
            % True Bearing with Sensor Noise
            bearing = atan2(dy, dx) - truePose(3) + normrnd(0,R(2,2));
            observations = [observations; i, r, bearing];
        end
    end
end