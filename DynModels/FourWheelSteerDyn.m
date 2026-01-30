function xk1 = FourWheelSteerDyn(xk, uk, Ts)
%FOURWHEELSTEERDYN: nlmpc receives the model of a point moving in the 2D
% plane where:
%
    theta = xk(3);
    M = [cos(theta), -sin(theta), 0; ...
         sin(theta), cos(theta), 0; ...
         0,        0,      1];
    xk1 = xk + M*uk*Ts;
end