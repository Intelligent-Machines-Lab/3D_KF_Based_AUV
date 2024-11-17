function dXdt = odefcn(t,X,U)
    y = X(9);
    p = X(8);
    r = X(7);

    cy = cos(y);
    sy = sin(y);
    cp = cos(p);
    sp = sin(p);
    cr = cos(r);
    sr = sin(r);
    % 
    % Ry = [cy sy 0;-sy cy 0;0 0 1];
    % Rp = [cp 0 -sp;0 1 0;sp 0 cp];
    % Rr = [1 0 0;0 cr sr;0 -sr cr];
    % 
    % R = Rr*Rp*Ry;
    % R = R';

    R = getRotation(y,p,r);
    
    A = [zeros(3) zeros(3) zeros(3) [0;0;1];
         eye(3)   zeros(3) zeros(3) zeros(3,1);
         zeros(3) zeros(3) zeros(3) zeros(3,1);
         zeros(1,3) zeros(1,3) zeros(1,3) 0;
         ];
   
   
   Rw = (1/cp)*[cp sr*sp cr*sp;0 cr*cp -sr*cp;0 sr cr];%
   %Rw = eye(3);

   B = [R        zeros(3);
         zeros(3) zeros(3);
         zeros(3) Rw;
         zeros(1,3) zeros(1,3)];


   dXdt = A*X + B*U;
end
