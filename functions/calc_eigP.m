function eig_P = calc_eigP(mat_Pk,mat_Pk_1)
    N = length(mat_Pk);
    eig_P = zeros(N,4);

    for ii=1:1:N
        Pk_1 = reshape(mat_Pk_1(ii,:),15,15);
        Pk   = reshape(mat_Pk(ii,:),15,15);
    
        [V, D] = eig(Pk_1);eig_Pk_1 = diag(D)';
        [V, D] = eig(Pk);   eig_Pk  = diag(D)';
        eig_P(ii,:) = [max(eig_Pk_1) min(eig_Pk_1) max(eig_Pk) min(eig_Pk)];
    end

end