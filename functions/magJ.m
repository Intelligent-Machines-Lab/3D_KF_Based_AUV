function A = magJ(W,M)
    wx_val = W(1);
    wy_val = W(2);
    wz_val = W(3);
    
    mx_val = M(1);
    my_val = M(2);
    mz_val = M(3);
    bx_val = M(4);
    by_val = M(5);
    bz_val = M(6);
    sx_val = M(7);
    sy_val = M(8);
    sz_val = M(9);
    
    % wx = sym('wx');
    % wy = sym('wy');
    % wz = sym('wz');
    % 
    % bx = sym('bx');
    % by = sym('by');
    % bz = sym('bz');
    % 
    % sx = sym('sx');
    % sy = sym('sy');
    % sz = sym('sz');
    % 
    % mx = sym('mx');
    % my = sym('my');
    % mz = sym('mz');
    % 
    % S = [0 -wz wy;wz 0 -wx;-wy wx 0];
    % A = [sx 0 0; 0 sy 0;0 0 sz];
    % 
    % T = inv(A)*S*A;
    % dot_m = -T*[mx my mz]' + T*[bx by bz]';
    % 
    % 
    % fmx = dot_m(1);
    % fmy = dot_m(2);
    % fmz = dot_m(3);
    % df_dm = [diff(fmx,mx) diff(fmx,my) diff(fmx,mz);diff(fmy,mx) diff(fmy,my) diff(fmy,mz);diff(fmz,mx) diff(fmz,my) diff(fmz,mz)];
    % 
    % df_db = [diff(fmx,bx) diff(fmx,by) diff(fmx,bz);diff(fmy,bx) diff(fmy,by) diff(fmy,bz);diff(fmz,bx) diff(fmz,by) diff(fmz,bz)];
    % df_ds = [diff(fmx,sx) diff(fmx,sy) diff(fmx,sz);diff(fmy,sx) diff(fmy,sy) diff(fmy,sz);diff(fmz,sx) diff(fmz,sy) diff(fmz,sz)];
    % 
    % DF = [df_dm df_db df_ds];
    
    
    wx = wx_val;
    wy = wy_val;
    wz = wz_val;

    mx = mx_val;
    my = my_val;
    mz = mz_val;

    bx = bx_val;
    by = by_val;
    bz = bz_val;

    sx = sx_val;
    sy = sy_val;
    sz = sz_val;
    
 DF=[0,(sy*wz)/sx,-(sz*wy)/sx,0,-(sy*wz)/sx,(sz*wy)/sx,(sy*wz*(by))/sx^2-(sz*wy*(bz))/sx^2-(sy*wz*(my))/sx^2+(sz*wy*(mz))/sx^2,(wz*(my))/sx-(wz*(by))/sx,(wy*(bz))/sx-(wy*(mz))/sx;
 -(sx*wz)/sy,0,(sz*wx)/sy,(sx*wz)/sy,0,-(sz*wx)/sy,(wz*(bx))/sy-(wz*(mx))/sy,(sz*wx*(bz))/sy^2-(sx*wz*(bx))/sy^2+(sx*wz*(mx))/sy^2-(sz*wx*(mz))/sy^2,(wx*(mz))/sy-(wx*(bz))/sy;
 (sx*wy)/sz,-(sy*wx)/sz,0,-(sx*wy)/sz,(sy*wx)/sz,0,(wy*(mx))/sz-(wy*(bx))/sz,(wx*(by))/sz-(wx*(my))/sz,(sx*wy*(bx))/sz^2-(sy*wx*(by))/sz^2-(sx*wy*(mx))/sz^2+(sy*wx*(my))/sz^2];

    A = [(DF);zeros(6,9)];
    %Ad = expm(A*0.1)

    %return
end