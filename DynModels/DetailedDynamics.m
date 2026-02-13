function xk = DetailedDynamics(x, u, Ts)
    omegaF = u(1);
    omegaR = u(2);
    phiF = u(3);
    phiR = u(4);

    R = 0.1;
    Lf = 0.3;
    Lr = 0.25;

    vx = (R/2) * (omegaF * cos(phiF) + omegaR * cos(phiR));
    vy = (R/2) * (omegaF * sin(phiF) + omegaR * sin(phiR));
    omega = (R/(Lf+Lr)) * (omegaF * sin(phiF) - omegaR * sin(phiR));

    uk = [vx;vy;omega];

    theta = x(3);
    M = [cos(theta), -sin(theta), 0; ...
         sin(theta), cos(theta), 0; ...
         0,        0,      1];
    xk = x + M*uk*Ts;
end