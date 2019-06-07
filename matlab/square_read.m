close all
clear

port='/dev/ttyACM0';
instr=instrfind('port',port,'Status','open');
if size(instr)~=0
    fclose(instr);
end
a = serial(port);
fopen(a);


figure()
xlim([0 128]);
hold on

colors='krgb';
for in=1:4
    sca{in}=scatter([],[],colors(in),'filled');
end


disp('Reset now')
pause(1);

amps=1:127;
ws=nan(4,length(amps));
index=1;
while (true)
    data=[];
    in=1;
    while(true)
        s=fgetl(a);
        disp(s);
        if isempty(s)
            continue %try again
        end
        if contains(s,'END')
            b_end=true;
            break
        end
        if contains(s,'NEXT')
            b_end=false;
            break
        end
        n=str2num(s);
        data(:,in)=n';
        in=in+1;
    end
    index=abs(data(1,end));
    no_clear=1;
    identify_motor
    for j=1:4
        ws(j,index)=w{j};
        set(sca{j},'XData',amps(1:index),...
                   'YData',ws(j,1:index));
        ylim([0,max(ws,[],'all')+0.5]);
        drawnow()
    end
    if b_end
        break
    end
end