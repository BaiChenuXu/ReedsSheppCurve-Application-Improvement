% visualizaiton of the calculate result
function vehAnimation(x,y,theta,cfg,veh)
sz=get(0,'screensize');
figure('outerposition',sz);
videoFWriter = VideoWriter('ReedsSheppTraj.mp4','MPEG-4');
open(videoFWriter);

% scatter(ObstList(:,1),ObstList(:,2),10,'r')
hold on
axis equal
xlim([cfg.minX,cfg.maxX]);
ylim([cfg.minY,cfg.maxY]);
xlabel('x [m]')
ylabel('y [m]')  
title("Reeds-Shepp Trajectory");
box on
% plot planning trajectory
plot(x,y,'b')
px = x(1);
py = y(1);
pth = theta(1);
% get vehicle frame pose according to the pose of the center of rear shaft
[vehx,vehy] = getVehTran(px,py,pth,veh);
% vehicle frame
h1 = plot(vehx,vehy,'k');
% the center of rear shaft
h2 = plot(px,px,'rx','MarkerSize',10);
img = getframe(gcf);
writeVideo(videoFWriter,img);
for i = 2:length(theta)
    px = x(i);
    py = y(i);
    pth = theta(i);
    [vehx,vehy] = getVehTran(px,py,pth,veh);
    % update the h1 figure handle，add the x of vehicle frame
    h1.XData = vehx;
    h1.YData = vehy;
    % update the h2 figure handle，add the y of vehicle frame
    h2.XData = px;
    h2.YData = py;
    img = getframe(gcf);
    writeVideo(videoFWriter,img);
    %         pause(0.005)
end
close(videoFWriter);
end

% get the pose of rear shaft according to the pose of the center of rear shaft
function [x,y] = getVehTran(x,y,theta,veh)
W = veh.width;
LF = veh.lf;
LB = veh.lb;

% 4 corner points determine the vehicle frame
Cornerfl = [LF, W/2];
Cornerfr = [LF, -W/2];
Cornerrl = [-LB, W/2];
Cornerrr = [-LB, -W/2];
% the position of the center of rear shaft
Pos = [x,y];
% calculate rotation matrix of the 4 corner point
% convert the angle to the direction cosin matrix
dcm = angle2dcm(-theta, 0, 0);
% rotation transformation
tvec = dcm*[Cornerfl';0];
tvec = tvec';
% translation transformation
Cornerfl = tvec(1:2)+Pos;

tvec = dcm*[Cornerfr';0];
tvec = tvec';
Cornerfr = tvec(1:2)+Pos;

tvec = dcm*[Cornerrl';0];
tvec = tvec';
Cornerrl = tvec(1:2)+Pos;

tvec = dcm*[Cornerrr';0];
tvec = tvec';
Cornerrr = tvec(1:2)+Pos;

%
% feedback the x and y position of the 4 corner points of vehicle frame
x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end