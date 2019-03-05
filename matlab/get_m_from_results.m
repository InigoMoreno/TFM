clear
%close all
data_files={'results_jacked.mat'
            'results_ground_x.mat'
            'results_ground_y.mat'};
colors='bgr';
figure();
for j=1:4
    ax{j}=subplot(2,2,j);
    hold on;
    ylabel(['M' num2str(j) ' [N m]'])
    xlabel(['w' num2str(j) ' [rad/s]'])
end
for i=1:3
    load(data_files{i},'Ks','amps');
    if i==3
        Ks([1 4],:)=-Ks([1 4],:);
    end
    
    for j=1:4
        filter=1:find(~isnan(Ks(j,:)),1,'last');
        us=amps(filter)/127;

        u=symunit;
        wjs=Ks(j,filter).*us*60/(2*pi) *u.rad/u.s ; %W in rad/s
        Vs=us*24*u.V; %V in volts

        Ke=24*u.V/(7000*u.rpm);
        Km=4250*u.g*u.cm  *  9.81*u.m/u.s^2 / (13*u.A);
        R=24*u.V/(13*u.A);

        Ke=unitConvert(Ke,'SI','derived');
        Km=unitConvert(Km,'SI','derived');
        R=unitConvert(R,'SI','derived');

        Mjs=(Vs-Ke*wjs)/(R/Km);
        line(ax{j},double(separateUnits(wjs)),double(separateUnits(Mjs)),'Color',colors(i))
    end
end