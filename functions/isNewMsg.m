function [idx,is_new] = isNewMsg(data,ti,tf)
     is_new = false;
     idx = find(data.Time > ti & data.Time < tf);
     if ~isempty(idx)
         is_new = true;
     end
end