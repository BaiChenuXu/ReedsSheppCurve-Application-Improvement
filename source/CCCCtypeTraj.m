function [isok,path,costFinal] = CCCCtypeTraj(x,y,phi,start,endp,veh,Types)
costFinal = inf;
type = repmat('N',[1,5]);
path = RSpathStruct(type,0,0,0,0,0);
[isok,t,u,v] = LpRupLumRm(x,y,phi);
% according to the formula 8.7 in the paper to calculate L+R+L-R-
if isok
    pathTemp = RSpathStruct(Types(3,:),t,u,-u,v,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,u,-u,v,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(3,:),t,u,-u,v,0);
    end
end
% according to the formula 8.7 in the paper and timeflip symmetry to calculate L-R-L+R+
[isok,t,u,v] = LpRupLumRm(-x,y,-phi);
if isok
    pathTemp = RSpathStruct(Types(3,:),-t,-u,u,-v,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,u,-u,v,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(3,:),-t,-u,u,-v,0);
    end
end
% according to the formula 8.7 in the paper and reflect symmetry to calculate R+L+R-L-
[isok,t,u,v] = LpRupLumRm(x,-y,-phi);
if isok
    pathTemp = RSpathStruct(Types(4,:),t,u,-u,v,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,u,-u,v,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(4,:),t,u,-u,v,0);
    end
end
% according to the formula 8.7 in the paper and timeflip + reflect symmetry to calculate R-L-R+L+
[isok,t,u,v] = LpRupLumRm(-x,-y,phi);
if isok
    pathTemp = RSpathStruct(Types(1,:),-t,-u,u,-v,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-t,-u,u,-v,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(4,:),-t,-u,u,-v,0);
    end
end
[isok,t,u,v] = LpRumLumRp(x,y,phi);
% according to the formula 8.8 in the paper to calculate L+R-L-R+
if isok
    pathTemp = RSpathStruct(Types(3,:),t,-u,-u,v,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,-u,-u,v,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(3,:),t,-u,-u,v,0);
    end
end
[isok,t,u,v] = LpRumLumRp(-x,y,-phi);
% according to the formula 8.8 in the paper and timeflip symmetry to calculate L-R+L+R-
if isok
    pathTemp = RSpathStruct(Types(3,:),-t,u,u,-v,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-t,u,u,-v,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(3,:),-t,u,u,-v,0);
    end
end
% according to the formula 8.8 in the paper and reflect symmetry to calculate R+L-R-L+
[isok,t,u,v] = LpRumLumRp(x,-y,-phi);
if isok
    pathTemp = RSpathStruct(Types(4,:),t,-u,-u,v,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,-u,-u,v,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(4,:),t,-u,-u,v,0);
    end
end
% according to the formula 8.8 in the paper and timeflip + reflect symmetry to calculate R-L+R+L-
[isok,t,u,v] = LpRumLumRp(-x,-y,phi);
if isok
    pathTemp = RSpathStruct(Types(4,:),-t,u,u,-v,0);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-t,u,u,-v,0);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(4,:),-t,u,u,-v,0);
    end
end
if costFinal == inf
    isok = false;
else
    isok = true;
end
end

% formula 8.7 in the paper, tauOmega() is formula 8.6
function [isok,t,u,v] = LpRupLumRm(x,y,phi)
xi = x+sin(phi);
eta = y-1-cos(phi);
rho = (2+sqrt(xi^2+eta^2))/4;
if rho >= 0 && rho <= 1
    u = acos(rho);
    [t,v] = tauOmega(u,-u,xi,eta,phi);
    % if t >= 0 && v <= 0 % the sign represent forward or backward
    isok = true;
    return
    % end
end
isok = false;
t = 0;
u = 0;
v = 0;
end

% formula 8.8 in the paper
function [isok,t,u,v] = LpRumLumRp(x,y,phi)
xi = x+sin(phi);
eta = y-1-cos(phi);
rho = (20-xi^2-eta^2)/16;
if rho >= 0 && rho <= 1
    u = -acos(rho);
    % if u >= pi/2
    [t,v] = tauOmega(u,u,xi,eta,phi);
    % if t >=0 && v >=0
    isok = true;
    return
    % end
    % end
end
isok = false;
t = 0;
u = 0;
v = 0;
end

% formula 8.6
function [tau,omega] = tauOmega(u,v,xi,eta,phi)
delta = limitAngleRange(u-v);
A = sin(u)-sin(delta);
B = cos(u)-cos(delta)-1;
t1 = atan2(eta*A-xi*B,xi*A+eta*B);
t2 = 2*(cos(delta)-2*cos(v)-2*cos(u))+3;
if t2 < 0
    tau = limitAngleRange(t1+pi);
else
    tau = limitAngleRange(t1);
end
omega = limitAngleRange(tau-u+v-phi);
end
