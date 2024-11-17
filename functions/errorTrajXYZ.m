function error = errorTrajXYZ(gt_traj,traj)

    gt_arrayTime = gt_traj(:,1);
    gt_size  = length(gt_arrayTime);
    gt_idx = 1;
    gt_newData = false;
    
    traj_arrayTime = traj(:,1);
    traj_size  = length(traj_arrayTime);
    traj_idx = 1;
    traj_newData = false;
    
    error = zeros(gt_size,5);

    trajXYZ = [0 0 0];
    gtXYZ   = [0 0 0];
    for ii = gt_arrayTime(1,1) : 0.01 : gt_arrayTime(end,1)
     
     data = gt_arrayTime(gt_idx);
     if ii > data
        gt_idx = gt_idx + 1;
        if gt_idx > gt_size
            gt_idx = gt_size;
        end
        gt_newData = true;
     end

     data = traj_arrayTime(traj_idx);
     if ii > data
        traj_idx = traj_idx + 1;
        if traj_idx > traj_size
            traj_idx = traj_size;
        end
        traj_newData = true;
     end
     
     if traj_newData
        traj_newData = false;
        trajXYZ = traj(traj_idx,2:end);
     end

     if gt_newData
        gt_newData   = false;
        gtXYZ = gt_traj(gt_idx,2:end);

        error(gt_idx,:) = [gt_traj(gt_idx,1) (gtXYZ(1) - trajXYZ(1)) (gtXYZ(2) - trajXYZ(2)) (gtXYZ(3) - trajXYZ(3)) norm(gtXYZ-trajXYZ)];
     end

    end
end