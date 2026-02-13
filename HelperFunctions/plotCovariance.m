function [XData, YData] = plotCovariance(stateEstimate, P)
    % Ellissi di covarianza per la posizione del robot
    posCov = P(1:2, 1:2);
    [eigVec, eigVal] = eig(posCov);
    angle = atan2(eigVec(2, 1), eigVec(1, 1));
    radii = 10*sqrt(diag(eigVal));
    theta = linspace(0, 2 * pi, 100);
    ellipse = [cos(theta); sin(theta)]' * diag(radii) * [cos(angle), -sin(angle); sin(angle), cos(angle)];
    XData = stateEstimate(1) + ellipse(:, 1);
    YData = stateEstimate(2) + ellipse(:, 2);
end