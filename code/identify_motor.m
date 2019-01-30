load randomimputs_nodeadband.mat
t=data(2,1:end-1)/1000;
d1=diff(data(4,:))./dt*60/49/20; %RPM
d2=-diff(data(5,:))./dt*60/49/20; %RPM
d3=diff(data(6,:))./dt*60/49/20; %RPM 
d4=-diff(data(7,:))./dt*60/49/20; %RPM

TT = timetable(milliseconds(data(2,:)'),data(4,:)',-data(5,:)',data(6,:)',-data(7,:)');

function TT_out = mult_timetable(TT_in, factor)
    t = TT_in.(TT_in.Properties.DimensionNames{1});
    v = TT_in.Variables*factor;
    TT_out=array2timetable(v,'RowTimes',t);
end

function TT_out = diff_timetable(TT_in)
    t = TT_in.(TT_in.Properties.DimensionNames{1});
    dt = diff(t);
    dv = diff(TT_in.Variables);
    new_t=(t(1:end-1)+t(2:end))/2;
    new_v=dv./seconds(repmat(dt,1,size(dv,2)));
    TT_out=array2timetable(new_v,'RowTimes',new_t);
end