function [isok,path,costFinal] = CCCtypeTraj(x,y,phi,start,endp,veh,Types)
costFinal= inf;
type = repmat('N',[1,5]);
path = RSpathStruct(type,0,0,0,0,0);
[isok,t,u,v] = LpRmL(x,y,phi);
% according to the formula 8.3 and 8.4 to calculate L+R-L+ and L+R-L-
if isok
    pathTemp = RSpathStruct(Types(1,:),t,u,v,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,u,v,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(1,:),t,u,v,0,0);
    end
end
% according to the formula 8.3 and 8.4 and timeflip symmetry to calculate L-R+L- and L-R+L+
[isok,t,u,v] = LpRmL(-x,y,-phi);
if isok
    pathTemp = RSpathStruct(Types(1,:),-t,-u,-v,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-t,-u,-v,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(1,:),-t,-u,-v,0,0);
    end
end
% according to the formula 8.3 and 8.4 and reflect symmetry to calculate R+L-R+ and R+L-R-
[isok,t,u,v] = LpRmL(x,-y,-phi);
if isok
    pathTemp = RSpathStruct(Types(2,:),t,u,v,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,u,v,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(2,:),t,u,v,0,0);
    end
end
% according to the formula 8.3 and 8.4 and timeflip + reflect symmetry to calculate R-L+R- and R-L+R+
[isok,t,u,v] = LpRmL(-x,-y,phi); %  reflect
if isok
    pathTemp = RSpathStruct(Types(2,:),-t,-u,-v,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-t,-u,-v,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(2,:),-t,-u,-v,0,0);
    end
end
% backwards
xb = x*cos(phi)+y*sin(phi);
yb = x*sin(phi)-y*cos(phi);
% according to the formula 8.3 and 8.4 to calculate L-R-L+
[isok,t,u,v] = LpRmL(xb,yb,phi);
if isok
    pathTemp = RSpathStruct(Types(1,:),v,u,t,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(v,u,t,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(1,:),v,u,t,0,0);
    end
end
% according to the formula 8.3 and 8.4 and timeflip symmetry to calculate L+R+L-
[isok,t,u,v] = LpRmL(-xb,yb,-phi);
if isok
    pathTemp = RSpathStruct(Types(1,:),-v,-u,-t,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-v,-u,-t,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(1,:),-v,-u,-t,0,0);
    end
end
% according to the formula 8.3 and 8.4 and reflect symmetry to calculate R-L-R+
[isok,t,u,v] = LpRmL(xb,-yb,-phi);
if isok
    pathTemp = RSpathStruct(Types(2,:),v,u,t,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(v,u,t,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(2,:),v,u,t,0,0);
    end
end
% according to the formula 8.3 and 8.4 and timeflip + reflect symmetry to calculate R+L+R-
[isok,t,u,v] = LpRmL(-xb,-yb,phi);
if isok
    pathTemp = RSpathStruct(Types(2,:),-v,-u,-t,0,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-v,-u,-t,0,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(2,:),-v,-u,-t,0,0);
    end
end
if costFinal == inf
    isok = false;
else
    isok = true;
end
end

% improvement formula 8.3/8.4
function [isok,t,u,v] = LpRmL(x,y,phi)
xi = x-sin(phi);
eta = y-1+cos(phi);
[theta,u1] = cart2pol(xi,eta);
if u1 <= 4
    u = -2*asin(u1/4);
    t = limitAngleRange(theta+u/2+pi);
    v = limitAngleRange(phi-t+u);
    % if t >= 0 && u <= 0
    isok = true;
    return
    % end
end
isok = false;
t = 0;
u = 0;
v = 0;
end
