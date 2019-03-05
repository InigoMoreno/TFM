close all
clear
load square_small.mat -mat data

x =data(1,:)/127; %input (-1 1)
t =data(2,:)/1000; %time (seconds)
m1=data(4,:)/49/20; %motor1 position (revolutions)

Ts=1/20;
TY=min(t):Ts:max(t);


%[Y,TY]=resample([x;m1]',t,1/Ts,'previous');
%xr=Y(:,1);
%m1r=Y(:,2);
xr=interp1(t,x,TY,'previous');
m1r=interp1(t,m1,TY,'linear');

xr=xr(:);
m1r=m1r(:);

n=numel(m1r);
ns = floor(n/2);
id=1:ns;
val=ns+1:n;

data_id=iddata(m1r(id),xr(id),Ts,'TimeUnit','seconds','TStart',min(TY(id)));
data_val=iddata(m1r(val),xr(val),Ts,'TimeUnit','seconds','TStart',min(TY(val)));