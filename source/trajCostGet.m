% calculate the trajectory cost
function costTot = trajCostGet(t1,t2,t3,t4,t5)
arr = [t1,t2,t3,t4,t5];
nonZeroArr = nonzeros(arr);
len = length(nonZeroArr);
costLen = 0;
reversePunish = 1.5;
for i = 1:len
    if nonZeroArr(i) < 0
        costLen = costLen + abs(nonZeroArr(i))*reversePunish;
    else
        costLen = costLen + abs(nonZeroArr(i));
    end
end
shiftPunish = 4;
costShift = 0;
for i = 1:len-1
    if sign(nonZeroArr(i) * nonZeroArr(i+1)) == -1
        costShift = costShift + shiftPunish;
    else

%     end
end
costTot = costLen + costShift;
end