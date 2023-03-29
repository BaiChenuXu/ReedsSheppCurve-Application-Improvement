function path = RSpathStruct(type,t,u,v,w,x)
path.type = type;
path.t = t;
path.u = u;
path.v = v;
path.w = w;
path.x = x;
path.totalLength = sum(abs([t,u,v,w,x]));
% end