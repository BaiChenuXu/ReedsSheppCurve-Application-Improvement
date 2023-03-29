% CSCC is the opposite direction of the CCSC
function [isok,path,costFinal] = CSCCtypeTraj(x,y,phi,start,endp,veh,Types)
costFinal = inf;
type = repmat('N',[1,5]);
path = RSpathStruct(type,0,0,0,0,0);
% backwards
xb = x*cos(phi)+y*sin(phi);
yb = x*sin(phi)-y*cos(phi);
% according to the formula 8.9 in the paper to calculate L-S-R-L+
[isok,t,u,v] = LpRmSmLm(xb,yb,phi);
if isok
    pathTemp = RSpathStruct(Types(7,:),v,u,-pi/2,t,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(v,u,-pi/2,t,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(7,:),v,u,-pi/2,t,0);
    end
end
% according to the formula 8.9 in the paper and timeflip symmetry to calculate L-S-R-L+
[isok,t,u,v] = LpRmSmLm(-xb,yb,-phi);
if isok
    pathTemp = RSpathStruct(Types(7,:),-v,-u,pi/2,-t,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-v,-u,pi/2,-t,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(7,:),-v,-u,pi/2,-t,0);
    end
end
% according to the formula 8.9 in the paper and reflect symmetry to calculate R-S-L-R+
[isok,t,u,v] = LpRmSmLm(xb,-yb,-phi);
if isok
    pathTemp = RSpathStruct(Types(8,:),v,u,-pi/2,t,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(v,u,-pi/2,t,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(8,:),v,u,-pi/2,t,0);
    end
end
% according to the formula 8.9 in the paper and timeflip + reflect symmetry to calculate R+S+L+R-
[isok,t,u,v] = LpRmSmLm(-xb,-yb,phi);
if isok
    pathTemp = RSpathStruct(Types(8,:),-v,-u,pi/2,-t,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-v,-u,pi/2,-t,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(8,:),-v,-u,pi/2,-t,0);
    end
end
% according to the formula 8.10 in the paper to calculate R-S-R-L+
[isok,t,u,v] = LpRmSmRm(xb,yb,phi);
if isok
    pathTemp = RSpathStruct(Types(11,:),v,u,-pi/2,t,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(v,u,-pi/2,t,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(11,:),v,u,-pi/2,t,0);
    end
end
% according to the formula 8.10 in the paper and to timeflip symmetry calculate R+S+R+L-
[isok,t,u,v] = LpRmSmRm(-xb,yb,-phi);
if isok
    pathTemp = RSpathStruct(Types(11,:),-v,-u,pi/2,-t,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-v,-u,pi/2,-t,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(11,:),-v,-u,pi/2,-t,0);
    end
end
% according to the formula 8.10 in the paper and to reflect symmetry calculate L-S-L-R+
[isok,t,u,v] = LpRmSmRm(xb,-yb,-phi);
if isok
    pathTemp = RSpathStruct(Types(12,:),v,u,-pi/2,t,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(v,u,-pi/2,t,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(12,:),v,u,-pi/2,t,0);
    end
end
% according to the formula 8.10 in the paper and to timeflip + reflect symmetry calculate L+S+L+R-
[isok,t,u,v] = LpRmSmRm(-xb,-yb,phi);
if isok
    pathTemp = RSpathStruct(Types(12,:),-v,-u,pi/2,-t,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-v,-u,pi/2,-t,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(12,:),-v,-u,pi/2,-t,0);
    end
end
if costFinal == inf
    isok = false;
else
    isok = true;
end
end

% formula 8.9 in the paper
function [isok,t,u,v] = LpRmSmLm(x,y,phi)
xi = x+sin(phi);
eta = y-1-cos(phi);
% xi = x-sin(phi);
% eta = y-1+cos(phi);
[theta,rho] = cart2pol(-eta,xi);
% [theta,rho] = cart2pol(xi,eta);
if rho >= 2
    [theta1,~] = cart2pol(sqrt(rho^2-4),-2);
    t = limitAngleRange(theta-theta1);
    u = 2-theta1;
    v = limitAngleRange(phi-pi/2-t);
    % if t >= 0 && u <= 0 && v <= 0
    isok = true;
    return
    % end
end
isok = false;
t = 0;
u = 0;
v = 0;
end

% formula 8.10 in the paper
function [isok,t,u,v] = LpRmSmRm(x,y,phi)
xi = x+sin(phi);
eta = y-1-cos(phi);
[theta,rho] = cart2pol(-eta,xi);
if rho >= 2
    t = theta;
    u = 2-rho;
    v = limitAngleRange(t+pi/2-phi);
    % if t >= 0 && u <= 0 && v <= 0
    isok = true;
    return
    % end
end
isok = false;
t = 0;
u = 0;
v = 0;
end

