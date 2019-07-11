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
    
    as=zeros(4,1);
    bs=zeros(4,1);
    for j=1:4
        ax{j}=subplot(2,2,j);
        hold on;
        title(['Motor ' num2str(j)])
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
        
        
        [~,KmUnits]=separateUnits(Km);
        Km=separateUnits(Ke)*KmUnits;
        
        
        N=49;
        eta=0.6;
        
        Mjs=eta*(N*Km*Vs-N^2*Ke*Km*wjs)/(R);
        x=wjs;
        y=to_units(Mjs,1/u.Newtonmeter,1);
        
        [xData,xUnits]=separateUnits(x);
        [yData,yUnits]=separateUnits(y);
        
        X=double(xData);
        Y=double(yData);
        f=fit(X',Y','a*x+b*sign(x)');
        %line(ax{j},double(separateUnits(wjs)),double(separateUnits(Mjs)),'Color',colors(i))
        plot(f,X,Y)
        xlabel(ax{j},'$\dot \varphi$ [rad/s]','Interpreter','latex','FontSize',12);
        ylabel(ax{j},'$\tau_{fric}$ [Nm]','Interpreter','latex','FontSize',12);
        as(j)=f.a;
        bs(j)=f.b;
    end
    disp(as);
    disp(bs);
    max_ylim=max(arrayfun(@(i)max(ylim(ax{i})),1:4));
    for j=1:4
        ylim(ax{j},[0 max_ylim]);
    end
    
 
end