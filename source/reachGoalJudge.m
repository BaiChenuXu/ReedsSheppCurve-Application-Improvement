% estimate if the trajecory can reach the end point
function logi_final = reachGoalJudge(traj_x,traj_y,traj_th,endp)
logi_1 = abs(traj_x(end) - endp(1)) < 0.2;
logi_2 = abs(traj_y(end) - endp(2)) < 0.2;
logi_3 = abs(traj_th(end) - endp(3)) < 0.2;
logi_final = logi_1 & logi_2 & logi_3;
end