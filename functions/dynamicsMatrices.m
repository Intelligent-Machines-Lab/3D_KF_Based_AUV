function [Ad,Bd] = dynamicsMatrices(y,p,r,T,method)

if method 
Ad = [1, 0, 0, 0, 0, 0, 0, 0, 0,       T*cos(p)*cos(y),            -T*(cos(r)*sin(y) - cos(y)*sin(p)*sin(r)),             T*(sin(r)*sin(y) + cos(r)*cos(y)*sin(p)), 0,                        0,                  0;
0, 1, 0, 0, 0, 0, 0, 0, 0,       T*cos(p)*sin(y),             T*cos(r)*cos(y) + T*sin(p)*sin(r)*sin(y),             T*cos(r)*sin(p)*sin(y) - T*cos(y)*sin(r), 0,                        0,                        0;
0, 0, 1, 0, 0, 0, 0, 0, 0,             -T*sin(p),                                      T*cos(p)*sin(r),                                      T*cos(p)*cos(r), 0,                        0,                        0;
T, 0, 0, 1, 0, 0, 0, 0, 0, (T^2*cos(p)*cos(y))/2,      -(T^2*(cos(r)*sin(y) - cos(y)*sin(p)*sin(r)))/2,       (T^2*(sin(r)*sin(y) + cos(r)*cos(y)*sin(p)))/2, 0,                        0,                        0;
0, T, 0, 0, 1, 0, 0, 0, 0, (T^2*cos(p)*sin(y))/2, (T^2*cos(r)*cos(y))/2 + (T^2*sin(p)*sin(r)*sin(y))/2, (T^2*cos(r)*sin(p)*sin(y))/2 - (T^2*cos(y)*sin(r))/2, 0,                        0,                        0;
0, 0, T, 0, 0, 1, 0, 0, 0,       -(T^2*sin(p))/2,                                (T^2*cos(p)*sin(r))/2,                                (T^2*cos(p)*cos(r))/2, 0,                        0,                        0;
0, 0, 0, 0, 0, 0, 1, 0, 0,                     0,                                                    0,                                                    0, T, (T*sin(p)*sin(r))/cos(p), (T*cos(r)*sin(p))/cos(p);
0, 0, 0, 0, 0, 0, 0, 1, 0,                     0,                                                    0,                                                    0, 0,                 T*cos(r),                -T*sin(r);
0, 0, 0, 0, 0, 0, 0, 0, 1,                     0,                                                    0,                                                    0, 0,        (T*sin(r))/cos(p),        (T*cos(r))/cos(p);
0, 0, 0, 0, 0, 0, 0, 0, 0,                     1,                                                    0,                                                    0, 0,                        0,                        0;
0, 0, 0, 0, 0, 0, 0, 0, 0,                     0,                                                    1,                                                    0, 0,                        0,                        0;
0, 0, 0, 0, 0, 0, 0, 0, 0,                     0,                                                    0,                                                    1, 0,                        0,                        0;
0, 0, 0, 0, 0, 0, 0, 0, 0,                     0,                                                    0,                                                    0, 1,                        0,                        0;
0, 0, 0, 0, 0, 0, 0, 0, 0,                     0,                                                    0,                                                    0, 0,                        1,                        0;
0, 0, 0, 0, 0, 0, 0, 0, 0,                     0,                                                    0,                                                    0, 0,                        0,                        1];
 
 
Bd = [T*cos(p)*cos(y),   -T*(cos(r)*sin(y) - cos(y)*sin(p)*sin(r)),    T*(sin(r)*sin(y) + cos(r)*cos(y)*sin(p)), 0,                        0,                        0;
  T*cos(p)*sin(y),    T*(cos(r)*cos(y) + sin(p)*sin(r)*sin(y)),   -T*(cos(y)*sin(r) - cos(r)*sin(p)*sin(y)), 0,                        0,                        0;
        -T*sin(p),                             T*cos(p)*sin(r),                             T*cos(p)*cos(r), 0,                        0,                        0;
T^2*cos(p)*cos(y), -T^2*(cos(r)*sin(y) - cos(y)*sin(p)*sin(r)),  T^2*(sin(r)*sin(y) + cos(r)*cos(y)*sin(p)), 0,                        0,                        0;
T^2*cos(p)*sin(y),  T^2*(cos(r)*cos(y) + sin(p)*sin(r)*sin(y)), -T^2*(cos(y)*sin(r) - cos(r)*sin(p)*sin(y)), 0,                        0,                        0;
      -T^2*sin(p),                           T^2*cos(p)*sin(r),                           T^2*cos(p)*cos(r), 0,                        0,                        0;
                0,                                           0,                                           0, T, (T*sin(p)*sin(r))/cos(p), (T*cos(r)*sin(p))/cos(p);
                0,                                           0,                                           0, 0,                 T*cos(r),                -T*sin(r);
                0,                                           0,                                           0, 0,        (T*sin(r))/cos(p),        (T*cos(r))/cos(p);
                0,                                           0,                                           0, 0,                        0,                        0;
                0,                                           0,                                           0, 0,                        0,                        0;
                0,                                           0,                                           0, 0,                        0,                        0;
                0,                                           0,                                           0, 0,                        0,                        0;
                0,                                           0,                                           0, 0,                        0,                        0;
                0,                                           0,                                           0, 0,                        0,                        0];
else

    cy = cos(y);
    sy = sin(y);
    cp = cos(p);
    sp = sin(p);
    cr = cos(r);
    sr = sin(r);

    Ry = [cy -sy 0;sy cy 0;0 0 1];
    Rp = [cp 0 sp;0 1 0;-sp 0 cp];
    Rr = [1 0 0;0 cr -sr;0 sr cr];
    
    R = Ry*Rp*Rr;


    Rw = (1/cp)*[cp sr*sp cr*sp;0 cr*cp -sr*cp;0 sr cr];%mirko

A = [zeros(3) zeros(3) zeros(3)  R        zeros(3);
     eye(3)   zeros(3) zeros(3)  zeros(3) zeros(3);
     zeros(3) zeros(3) zeros(3)  zeros(3) Rw;
     zeros(3) zeros(3) zeros(3)  zeros(3) zeros(3);
     zeros(3) zeros(3) zeros(3)  zeros(3) zeros(3)];



B = [R        zeros(3);
     zeros(3) zeros(3);
     zeros(3) Rw;
     zeros(3) zeros(3);
     zeros(3) zeros(3)];


    C = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 1 0 0 0 0 0 0 0 0 0 0 0 0];
    
    sysC=ss(A,B,C,0);
    sysD=c2d(sysC,T,'tustin');
    Ad=get(sysD,'a');
    Bd=get(sysD,'b');
end
end