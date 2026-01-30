function augPath = computeRefAngle(path, startPose)
%COMPUTEREFANGLE Summary of this function goes here
%   Detailed explanation goes here
angles = zeros(length(path),1);
angles(1) = startPose;

for i=2:length(path)
    angles(i) = atan2(path(i,2) - path(i-1,2), path(i,1) - path(i-1,1));
end
angles = unwrap(angles);
augPath = [path(:,1:2) angles]; 
end