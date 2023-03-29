% get the pose of step k+1，according to the control input and the pose of step k
function [x,y,theta] = VehKinematic(x,y,theta,D,delta,L)
% vehicle kinematic model
x = x+D*cos(theta);
y = y+D*sin(theta);
% calculate heading angle change
theta = theta+D/L*tan(delta);
theta = mod2pi(theta);
end

% limit the x range in the [-pi,pi]
function v = mod2pi(x)
v = rem(x,2*pi);
if v < -pi
    v = v+2*pi;
elseif v > pi
    v = v-2*pi;
end
end