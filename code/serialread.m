port='/dev/ttyACM1';
instr=instrfind('port',port,'Status','open');
if size(instr)~=0
    fclose(instr);
end
a = serial(port);
fopen(a);
data=[];
i=1;
while(true)
    s=fgetl(a);
    n=str2num(s);
    data(:,i)=n';
    i=i+1;
end
hold on

dt=diff(data(2,:))/1000;
x=data(1,1:end-1)/127;
t=data(2,1:end-1)/1000;
d1=diff(data(4,:))./dt*60/49/20/122;
d2=-diff(data(5,:))./dt*60/49/20/122;
d3=diff(data(6,:))./dt*60/49/20/122;
d4=-diff(data(7,:))./dt*60/49/20/122;


plot(x,smooth(d1))
plot(x,smooth(d2))
plot(x,smooth(d3))
plot(x,smooth(d4))
