% limit the x range in the [-pi,pi]
function v = limitAngleRange(x)
v = rem(x,2*pi);
if v < -pi
    v = v+2*pi;
elseif v > pi
    v = v-2*pi;
end
end