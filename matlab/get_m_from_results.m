clear
%close all
data_files={'results_jacked.mat'};
colors='bgr';
figure();
for j=1:4
    ax{j}=subplot(2,2,j);
    hold on;
    title(['Motor ' num2str(j)])
end
for i=1:length(data_files)
    load(data_files{i},'ws','amps');
    
    
    for j=1:4
        filter=~isnan(ws(j,:));
        us=amps(filter)/127;

        u=symunit;
        wjs=ws(j,filter)*u.rad/u.s ; %W in rad/s
        Vs=us*24.7*u.V; %V in volts

        Ke=24*u.V/(7000*u.rpm);
        Km=4250*u.g*u.cm  *  9.81*u.m/u.s^2 / (13*u.A);
        R=24*u.V/(13*u.A);

        Ke=unitConvert(Ke,'SI','derived');
        Km=unitConvert(Km,'SI','derived');
        R=unitConvert(R,'SI','derived');
        
        K=Ke;
        N=49;
        eta=0.6;
        
        Mjs=eta*(N*Km*Vs-N^2*Ke*Km*wjs)/(R);
        x=wjs;
        y=Mjs;
        
        [xData,xUnits]=separateUnits(x);
        [yData,yUnits]=separateUnits(y);
        
        %line(ax{j},double(separateUnits(wjs)),double(separateUnits(Mjs)),'Color',colors(i))
        line(ax{j},double(xData),double(yData),'Color',colors(i))
        xlabel(ax{j},symunit2str(xUnits(end)));
        ylabel(ax{j},symunit2str(yUnits(end))); 
    end
    max_ylim=max(arrayfun(@(i)max(ylim(ax{i})),1:4));
    for j=1:4
        ylim(ax{j},[0 max_ylim]);
    end
    
 
end