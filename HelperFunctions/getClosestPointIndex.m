function [distMin, idx] = getClosestPointIndex(reference, queryPoint)
    % reference: Nx3 matrix [x, y, theta]
    % queryPoint: 1x2 [x, y] or 1x3 [x, y, theta] vector
    
    refXY = reference(:, 1:2);
    ptXY = queryPoint(1:2);
    
    diffs = refXY - ptXY;
    distSq = sqrt(sum(diffs.^2, 2));
    
    % Find the index of the minimum distance
    [distMin, idx] = min(distSq);
end