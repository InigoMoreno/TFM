t=tcpclient('192.168.4.1',80);
read={'0','0','0','0'};
while true
    read = inputdlg({'Wheel1','Wheel2','Wheel3','Wheel4'},...
                        'Command', [1 7; 1 7; 1 7; 1 7],read);
    if isempty(read) || any(cellfun(@isempty,read))
        break;
    end
    try
        x=cellfun(@str2num,read);
    catch
        break
    end
    sum=0;
    disp(x)
    for i=1:4
        sum=sum+mod(x(i),2^8)*2^(8*(i-1));
    end
    write(t,uint32(sum))
end
delete(t)