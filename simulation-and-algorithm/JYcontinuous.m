function continuousData=JYcontinuous(axisAngle)

continuousData = axisAngle;

for i=2:length(axisAngle)
    delta = continuousData(i)-continuousData(i-1);
    if delta>100
        continuousData(i)=continuousData(i)-180;
    elseif delta<-100
        continuousData(i)=continuousData(i)+180;
    end
end

end