function Rypr = getRotation(y,p,r)

    cy = cos(y);
    sy = sin(y);

    cp = cos(p);
    sp = sin(p);
    
    cr = cos(r);
    sr = sin(r);

    Ry = [cy sy 0;-sy cy 0;0 0 1];
    
    Rp = [cp 0 -sp;0 1 0;sp 0 cp];
    
    Rr = [1 0 0;0 cr sr;0 -sr cr];
    
    Rypr = Rr*Rp*Ry;
    Rypr = Rypr';
end