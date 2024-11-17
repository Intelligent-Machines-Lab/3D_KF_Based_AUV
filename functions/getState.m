function X_INS = getState(X_INS,U_INS,T_KF)
    tspan = 0:T_KF/10:T_KF;
    X0_INS = [X_INS;-9.81];
    %opts = odeset('RelTol',1e-2,'AbsTol',1e-5,'MaxStep',0.001);
    [t,X] = ode45(@(t,X) odefcn(t,X,U_INS), tspan, X0_INS);        
    X_INS = X(size(X,1),1:9)';
end