if ~exist('no_clear','var') || ~no_clear
    close all
    clear
    load square_fast.mat -mat data
    no_clear=false;
end

t=data(2,:)/1000; %time(s)
x=data(1,:)/127;  %input (-1 1)
m=data(4:7,:)/49/20; %motor position (rev)

Ts=1/1000;
tr=min(t):Ts:max(t);

for i=1:size(m,1)
   % m(i,:)=smooth(t,m(i,:));
end

mr=interp1(t,m',tr,'spline')';
xr=interp1(t,x',tr,'previous')';


ds=diff(m,1,2)/Ts*60; %RPM
ts=conv(t,[.5 .5],'valid');
xs=interp1(t,x',ts,'previous')';

%hold on
%scatter(t,m(1,:)/max(m(1,:)))
%scatter(t,x/max(x))

%plot(tr,mr(1,:)/max(mr(1,:)))
%plot(tr,xr/max(xr))

%legend('m','x','mr','xr');

for i=1:4
    data_id{i}=iddata(mr(i,:)',xr,Ts,'TimeUnit','seconds','TStart',min(tr));
    sys{i}=procest(data_id{i},'P1I');
    if sys{i}.Report.Fit.FitPercent<90
        warning(['Fit of system is ' num2str(sys{i}.Report.Fit.FitPercent)])
    end
    disp(sys{i}.Kp*60);
end

%datar=interp1(data(2,:),data([1 3:end]),TY,'previous')';

%{
dt=diff(data(2,:))/1000;
x=data(1,1:end-1)/127;
t=data(2,1:end-1)/1000;
d1=diff(data(4,:))./dt*60/49/20; %RPM
d2=-diff(data(5,:))./dt*60/49/20; %RPM
d3=diff(data(6,:))./dt*60/49/20; %RPM 
d4=-diff(data(7,:))./dt*60/49/20; %RPM

TT = timetable(seconds(t)',d1',d2',d3',d4');

Ts=1/20;
TY=min(t):Ts:max(t);

% [Y,TY]=resample([x;d2]',t,1/Ts);
% xr=Y(:,1);
% d1r=Y(:,2);
xr=interp1(t,x,TY,'previous')';
d1r=interp1(t,d1,TY,'linear')';

n=numel(d1r);
ns = floor(n/2);
id=1:ns;
val=ns+1:n;

data_id=iddata(d1r(id),xr(id),Ts,'TimeUnit','seconds','TStart',min(TY(id)));
data_val=iddata(d1r(val),xr(val),Ts,'TimeUnit','seconds','TStart',min(TY(val)));
%}

%{
sys=ssest(data_id,1,'Form','modal','DisturbanceModel','none');
compare(sys,data_val,20) 

syms t x(t) y(t) u(t)

A=sys.A;
B=sys.B;
C=sys.C;
D=sys.D;

eq1=formula(diff(x)==A*x+B*u);
eq2=formula(y==C*x+D*u);
eq1=subs(eq1,x,(y-D*u)/C);
eq1=eq1/coeffs(lhs(eq1));
pretty(vpa(eq1,3));
pretty(vpa(eq2,3));

%}