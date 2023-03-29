function [isok,path,costFinal] = CSCtypeTraj(x,y,phi,start,endp,veh,Types)
costFinal = inf;
type = repmat('N',[1,5]);
path = RSpathStruct(type,0,0,0,0,0);
% according to the formula 8.1 in the paper to calculate L+S+L+
[isok,t,u,v] = LpSpLp(x,y,phi);
if isok
    pathTemp = RSpathStruct(Types(15,:),t,u,v,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,u,v,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(15,:),t,u,v,0,0);
    end
end
% according to the formula 8.1 in the paper and timeflip symmetry to calculate L-S-L-
[isok,t,u,v] = LpSpLp(-x,y,-phi);
if isok
    pathTemp = RSpathStruct(Types(15,:),-t,-u,-v,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-t,-u,-v,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(15,:),-t,-u,-v,0,0);
    end
end
% according to the formula 8.1 in the paper and reflect symmetry to calculate R+S+R+
[isok,t,u,v] = LpSpLp(x,-y,-phi);
if isok
    pathTemp = RSpathStruct(Types(16,:),t,u,v,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,u,v,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(16,:),t,u,v,0,0);
    end
end
% according to the formula 8.1 in the paper and timeflp + reflect symmetry to calculate R-S-R-
[isok,t,u,v] = LpSpLp(-x,-y,phi);
if isok
    pathTemp = RSpathStruct(Types(16,:),-t,-u,-v,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-t,-u,-v,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(16,:),-t,-u,-v,0,0);
    end
end
% according to the formula 8.1 in the paper calculate L+S+R+
[isok,t,u,v] = LpSpRp(x,y,phi);
if isok
    pathTemp = RSpathStruct(Types(13,:),t,u,v,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,u,v,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(13,:),t,u,v,0,0);
    end
end
% according to the formula 8.2 in the paper and timeflp symmetry to calculate L-S-R-
[isok,t,u,v] = LpSpRp(-x,y,-phi);
if isok
    pathTemp = RSpathStruct(Types(13,:),-t,-u,-v,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-t,-u,-v,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(13,:),-t,-u,-v,0,0);
    end
end
% according to the formula 8.2 in the paper and reflect symmetry to calculate R+S+L+
[isok,t,u,v] = LpSpRp(x,-y,-phi);
if isok
    pathTemp = RSpathStruct(Types(14,:),t,u,v,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,u,v,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(14,:),t,u,v,0,0);
    end
end
% according to the formula 8.2 in the paper and timeflip + reflect symmetry to calculate R-S-L-
[isok,t,u,v] = LpSpRp(-x,-y,phi);
if isok
    pathTemp = RSpathStruct(Types(14,:),-t,-u,-v,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-t,-u,-v,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(14,:),-t,-u,-v,0,0);
    end
end
if costFinal == inf
    isok = false;
else
    isok = true;
end
end

% formula 8.1
function [isok,t,u,v] = LpSpLp(x,y,phi)
[t,u] = cart2pol(x-sin(phi),y-1+cos(phi));
% if t >= 0
v = limitAngleRange(phi-t);
% if t >= 0 && u >= 0 && v >= 0 % the sign represent forward or backward
isok = true;
return
% end
% end
% isok = false;
% t = 0;
% u = 0;
% v = 0;
end

% formula 8.2
function [isok,t,u,v] = LpSpRp(x,y,phi)
[t1,u1] = cart2pol(x+sin(phi),y-1-cos(phi));
if u1^2 >= 4
    u = sqrt(u1^2-4);
    [theta,~]= cart2pol(u,2);
    % theta = atan2(2,u);
    t = limitAngleRange(t1+theta);
    v = limitAngleRange(t-phi);
    % if t >= 0 && v >= 0 % the sign represent forward or backward
    isok = true;
    return
    % end
end
isok = false;
t = 0;
u = 0;
v = 0;
end
