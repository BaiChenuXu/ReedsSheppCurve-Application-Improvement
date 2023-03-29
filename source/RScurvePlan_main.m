% This demo represent how to use the ReedsShepp Curve to plan the trajectory
% from one pose to another pose on the no obstacles map

% This demo extract and reorganize the relevant code that is from a hybrid
% A star algorithm demo (download from the https://github.com/wanghuohuo0716/hybrid_A_star)

% At the same time，make the 7 modifications：

% 1）extract the code that complete the "CSCC" calculation，and create a single
%   function to accommodate them. that make the struct of the program more clarity

% 2) when select the trajectory from the several candidate trajectory，not only
%    consider the length of trajectory，but also focus on the back driving，
%    the switch from forward to backward，and so on. make use the cost function
%    to punish the focused vehicle behavior and select the optimization trajectory

% 3）when use the above modifications，the limit of trjectory direction at every
%    function that accomplish the calculation in the reference paper will decrease
%    the optional trajectory number，so delete the limit

% 4）after applying the 3), some candidate trajectory conld not reach the goal
%    so, add a funcion to estimate if the trajectory can make the vehicle
%    reach the goal, if not, delete the trajectory

% 5) the calculation of some code accomplished can get right result, but the specific
%    formula is different from the formula of reference paper. And the formula in
%    reference paper can also get the right result. if the above situation appear
%    use the formula in the reference paper.

% 6) regrading to the "CCCC" type trajectory, when generate the "path", the sign
%    of "t", "u", "v" (which is subtrajectory length) have some error, so
%    make correction

% 7) change the path type from the class type to the struct type

% reference paper：OPTIMAL PATHS FOR A CAR THAT GOES BOTH FORWARDS AND BACKWARDS
% author：J. A. REEDS AND L. A. SHEPP

% reference book: Planning Algorithms (by LaValle), page 884

clc
clear

% set the map boundary
mapBound_up=50;
mapBound_down=0;
mapBound_left=0;
mapBound_right=80;

cfg.minX = mapBound_left;
cfg.maxX = mapBound_right;
cfg.minY = mapBound_down;
cfg.maxY = mapBound_up;

obstlist = [];
for i = mapBound_left:mapBound_right
    obstlist(end+1,:) = [i,mapBound_up];   %#ok<*SAGROW>
end
for i = mapBound_left:mapBound_right
    obstlist(end+1,:) = [i,mapBound_down];
end
for i = mapBound_down:mapBound_up
    obstlist(end+1,:) = [mapBound_left,i];
end
for i = mapBound_down:mapBound_up
    obstlist(end+1,:) = [mapBound_right,i];
end
cfg.obst = obstlist;

% set the start and end pose of the vehicle
start = [20, 30, 0];
endp = [25, 20, pi/2];

% set vehicle parameters
veh.wb = 3.7;
veh.width = 2.6;
veh.lf = 4.5;
veh.lb = 1.0;
veh.maxSteer = 0.6;
veh.minRadius = veh.wb/tan(veh.maxSteer);

% coordinate convert
pvec = endp-start;
x = pvec(1);
y = pvec(2);
phi = start(3);
phi = limitAngleRange(phi);
dcm = angle2dcm(phi, 0, 0);
tvec = dcm * [x; y ; 0];
x = tvec(1);
y = tvec(2);

smax = veh.maxSteer;
rmin = veh.minRadius;

% trajectory type define
Types = [
    'L', 'R', 'L', 'N', 'N' ;           %1
    'R', 'L', 'R', 'N', 'N' ;           %2
    'L', 'R', 'L', 'R', 'N' ;           %3
    'R', 'L', 'R', 'L', 'N' ;           %4
    'L', 'R', 'S', 'L', 'N' ;           %5
    'R', 'L', 'S', 'R', 'N' ;           %6
    'L', 'S', 'R', 'L', 'N' ;           %7
    'R', 'S', 'L', 'R', 'N' ;           %8
    'L', 'R', 'S', 'R', 'N' ;           %9
    'R', 'L', 'S', 'L', 'N' ;           %10
    'R', 'S', 'R', 'L', 'N' ;           %11
    'L', 'S', 'L', 'R', 'N' ;           %12
    'L', 'S', 'R', 'N', 'N' ;           %13
    'R', 'S', 'L', 'N', 'N' ;           %14
    'L', 'S', 'L', 'N', 'N' ;           %15
    'R', 'S', 'R', 'N', 'N' ;           %16
    'L', 'R', 'S', 'L', 'R' ;           %17
    'R', 'L', 'S', 'R', 'L'             %18
    ];

% Reeds Sheep trajectory calculate
path = FindRSPath_new(x,y,pvec(3),start,endp,veh,Types);

% get the discret point of the trajectory
[traj_x,traj_y,traj_th] = trajPointGet(path,start,veh);

% visualizaiton of the calculate result
vehAnimation(traj_x,traj_y,traj_th,cfg,veh)

function path = FindRSPath_new(x,y,phi,start,endp,veh,Types)
rmin = veh.minRadius;
x = x/rmin;
y = y/rmin;

% traverse 6 methods and select the minimum cost method
[isok1,path1,costFinal1] = CSCtypeTraj(x,y,phi,start,endp,veh,Types);
[isok2,path2,costFinal2] = CCCtypeTraj(x,y,phi,start,endp,veh,Types);
[isok3,path3,costFinal3] = CCCCtypeTraj(x,y,phi,start,endp,veh,Types);
[isok4,path4,costFinal4] = CCSCtypeTraj(x,y,phi,start,endp,veh,Types);
[isok5,path5,costFinal5] = CSCCtypeTraj(x,y,phi,start,endp,veh,Types);
[isok6,path6,costFinal6] = CCSCCtypeTraj(x,y,phi,start,endp,veh,Types);
isoks = [isok1, isok2, isok3, isok4, isok5, isok6];
paths = {path1, path2, path3, path4, path5, path6};
costFinals = [costFinal1, costFinal2, costFinal3, costFinal4,...
    costFinal5, costFinal6];

costTemp = inf;
for i = 1:6
    if isoks(i) == true
        elem = paths{i};
        if costTemp > costFinals(i)
            costTemp = costFinals(i);
            path = elem;
        end
    end
end
end

