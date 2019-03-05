%% ROS Setup for MATLAB

% --- FOR MULTI COMPUTER ---
% Enter the local ip address here on the ROS network
% Remember to edit ~/.bashrc with the local ip for export ROS_IP on host computer.
% Also edit ~/.bashrc on remote pc

ROS_MASTER_IP = '192.168.1.109';
ROS_IP = '192.168.1.105';

% --- FOR SINGLE COMPUTER ---
ROS_MASTER_IP = '127.0.0.1';
ROS_IP = '127.0.0.1';

a=dbstack;
a=a(end);

setenv('ROS_MASTER_URI', ['http://',ROS_MASTER_IP,':11311']);
setenv('ROS_IP', ROS_IP);
setenv('ROS_HOSTNAME', ROS_IP); % This is required on the visualisation computer
if ~robotics.ros.internal.Global.isNodeActive
    try
        fprintf('[%s] ROS: Attempting ROS init... \n',datestr(now,'HH:MM:SS'));
        rosinit('NodeName',a.name);

    catch
        fprintf('[%s] ROS: ROS will be reinitialised \n',datestr(now,'HH:MM:SS'));
        rosshutdown
        rosinit;
    end
else
    fprintf('[%s] ROS: ROS node active \n',datestr(now,'HH:MM:SS'));
end