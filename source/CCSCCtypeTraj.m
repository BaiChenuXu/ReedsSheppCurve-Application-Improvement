function [isok,path,costFinal] = CCSCCtypeTraj(x,y,phi,start,endp,veh,Types)
costFinal = inf;
type = repmat('N',[1,5]);
path = RSpathStruct(type,0,0,0,0,0);
[isok,t,u,v] = LpRmSLmRp(x,y,phi);
% according to the formula 8.11 to calculate L+R-S-L-R+
if isok
    pathTemp = RSpathStruct(Types(17,:),t,-pi/2,u,-pi/2,v);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,-pi/2,u,-pi/2,v);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(17,:),t,-pi/2,u,-pi/2,v);
    end
end
% according to the formula 8.11 and timeflip symmetry to calculate L+R-S-L-R+
[isok,t,u,v] = LpRmSLmRp(-x,y,-phi);
if isok
    pathTemp = RSpathStruct(Types(17,:),-t,pi/2,-u,pi/2,-v);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-t,pi/2,-u,pi/2,-v);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(17,:),-t,pi/2,-u,pi/2,-v);
    end
end
% according to the formula 8.11 and reflect symmetry to calculate R+L-S-R-L+
[isok,t,u,v] = LpRmSLmRp(x,-y,-phi);
if isok
    pathTemp = RSpathStruct(Types(18,:),t,-pi/2,u,-pi/2,v);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(t,-pi/2,u,-pi/2,v);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(18,:),t,-pi/2,u,-pi/2,v);
    end
end
% according to the formula 8.11 and timeflip + reflect symmetry to calculate R-L+S+R+L-
[isok,t,u,v] = LpRmSLmRp(-x,-y,phi);
if isok
    pathTemp = RSpathStruct(Types(18,:),-t,pi/2,-u,pi/2,-v);
    [traj_x,traj_y,traj_th] = trajPointGet(pathTemp,start,veh);
    logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp);
    costTot = trajCostGet(-t,pi/2,-u,pi/2,-v);
    if costFinal > costTot && logi_final == 1
        costFinal = costTot;
        path = RSpathStruct(Types(18,:),-t,pi/2,-u,pi/2,-v);
    end
end
if costFinal == inf
    isok = false;
else
    isok = true;
end
end

% improvement formula 8.11 
function [isok,t,u,v] = LpRmSLmRp(x,y,phi)
xi = x+sin(phi);
eta = y-1-cos(phi);
[~,rho] = cart2pol(xi,eta);
if rho >= 2
    u = 4-sqrt(rho^2-4);
    % if u <= 0
    t = limitAngleRange(atan2((4-u)*xi-2*eta,-2*xi+(u-4)*eta));
    v = limitAngleRange(t-phi);
    % if t >= 0 && v >= 0
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

% the formula 8.11 in the paper
% formula 8.11
% function [isok,t,u,v] = LpRmSLmRp(x,y,phi)
%     xi = x+sin(phi);
%     eta = y-1-cos(phi);
%     [theta,rho] = cart2pol(xi,eta);
%     if rho >= 2
%         t = limitAngleRange(theta - acos(-2/rho));
%         if t > 0
%             u = 4 - (xi + 2 * cos(t))/sin(t);
%             v = limitAngleRange(t-phi);
% %           if t > 0 && v >= 0
%               isok = true;
%               return
% %           end
%         end
%     end
%     isok = false;
%     t = 0;
%     u = 0;
%     v = 0;
% end