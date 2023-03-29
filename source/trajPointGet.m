% get discrete point of trajectory
function [traj_x,traj_y,traj_th] = trajPointGet(path,start,veh)
mres = 0.1;
smax = veh.maxSteer;
rmin = veh.minRadius;
types = path.type;
t = rmin*path.t;
u = rmin*path.u;
v = rmin*path.v;
w = rmin*path.w;
x = rmin*path.x;
segs = [t,u,v,w,x];
traj_x = start(1);
traj_y = start(2);
traj_th = start(3);

% according to the ReedsShepp planning trajectory，the resolution of the vehicle motion
% and vehicle kinematic model to calculate vehicle real trajectory
for i = 1:5
    if segs(i) ==0
        continue
    end
    s = sign(segs(i));
    D = s*mres;
    if types(i) == 'S'
        delta = 0;
    elseif types(i) == 'L'
        delta = smax;
    elseif types(i) == 'R'
        delta = -smax;
    else
        % do nothing
    end
    for idx = 1:round(abs(segs(i))/mres)
        [px,py,pth] = vehKinematic(traj_x(end),traj_y(end),traj_th(end),D,delta,veh.wb);
        traj_x = [traj_x, px]; %#ok<*AGROW>
        traj_y = [traj_y, py];
        traj_th = [traj_th, pth];
    end
end
end