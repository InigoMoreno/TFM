%% UAVP run_visual.m
% Visual GUI that must run in parallel to run_controller to visualise the 
% runtime data and send setpoint commands

close all force; clear all force; clc;

addpath(pwd, 'GUI')

fprintf('[%s] Starting UAVP run_visual.m \n',datestr(now,'HH:MM:SS'));

%% Initialise and load variables into workspace
setupROS

initialise

%% Intialise CGraphic communicator

GraphicCom = CGraphic(false,cfg.Graphic.nQuadStates, cfg.Graphic.nQuadSetpoints, cfg.Graphic.nQuadMPCPlan, cfg.Graphic.nObsStates, share);

%% Initialise GUI
fprintf('[%s] Opening GUI \n',datestr(now,'HH:MM:SS'));
% Create GUI
ControlGUI = gui(share,setpointsQuadrotor);
movegui(ControlGUI,'north');

% Get all handles from ControlGUI
handles = guidata(ControlGUI);
% Get 3D plot and 2D subplot handles
h3d_plot = handles.fig_main;   

% Show automated setpoint
vis_set = scatter3(h3d_plot,0,0,0,400,'m.');

%% Generate quadrotor and obstacle visualisations
for i = 1:model.nSys
    ids=[];
    for j = 1:model.nQuad
        ids(j)=mapping(model.nQuad*(i-1)+j);
    end
    VisualQuad(i) = CVisualQuadPL(h3d_plot, ids, share, 0.4);
end

if model.nObs == 0
    VisualObs = CVisualObs.empty(0,0);
end
for i = 1:model.nObs
    VisualObs(i) = CVisualObs(h3d_plot, i, share, obsDim(:,i)', obsBuffers(2,i), obsBuffers(1,i));
end

%%  =======================================================================
%% MAIN LOOP

%%% Timer
cnt = 0;   % Number of visual loops performed
t_start = seconds(rostime('now'));
t_prev = t_start;

while(true)
    % Flags    
    exitflag = 0;
    infeasible = 0;
    
    %% Timer
    t_now = seconds(rostime('now'));
    t_dtmeasured = t_now - t_prev;
    
    if(t_dtmeasured > cfg.mindt)  
        
        % Timer and loop count
        t_prev = t_now;
        cnt = cnt + 1;
        
        if (mod(cnt,10) == 0 && cfg.verbose)
            fprintf('Frequency: %.3f Hz\n',1/t_dtmeasured)
        end
        
        %% Communicate with CGraphic / Get ROS data (subscribers)
        [G_Time, G_flagTime] = GraphicCom.getTime();
        
        [G_Controlmode,G_flagControlmode] = GraphicCom.getControlMode();
        
        [G_QuadrotorSetpoint, ~, G_flagnewQuadrotorSetpoint] = GraphicCom.getQuadrotorSetpoint();
        [G_QuadrotorState, G_flagQuadrotorState] = GraphicCom.getQuadrotorState();
        [G_MPCPlan, G_flagMPCPlan] = GraphicCom.getMPCPlan();
        
        [G_ObstacleState, G_flagObstacleState] = GraphicCom.getObstacleState(); 
        
        if G_flagObstacleState==2 %nObs is not the same as in intialise
            nVisual=length(VisualObs);
            newNObs = GraphicCom.model.nObs;
            if newNObs < nVisual %less obstacles than visual, just remove the obstacles
                for i=(newNObs+1):nVisual
                   VisualObs(i).delete() 
                end
                VisualObs=VisualObs(1:newNObs);
            elseif newNObs > nVisual %more obstacles than visual
                for i=nVisual+1:newNObs %for each new obstacle
                    if i<=size(obsDim,2) % if we have its size just create it again
                        VisualObs(i) = CVisualObs(h3d_plot, i, share, obsDim(:,i)', obsBuffers(2,i), obsBuffers(1,i));
                    else %we dont have its size
                        if size(obsDim,2)>0 %if there is at least one obstacle, replicate dimensions of last one
                            ni=size(obsDim,2);
                            VisualObs(i) = CVisualObs(h3d_plot, i, share, obsDim(:,ni)', obsBuffers(2,ni), obsBuffers(1,ni));
                        else %if there are no obstacles use hardcoded values
                            VisualObs(i) = CVisualObs(h3d_plot, i, share, [.4;.4;1.2]', .6, .1);
                        end
                    end
                end
            end
        end
        
        [G_PayloadPos, G_flagPayloadPos] = GraphicCom.getPayloadPos();
        
        %% Plotting and table update in GUI
        % If data is being received through ROS from controller start modifying GUI
        
        % Get current GUI data handles
        try
            handles = guidata(ControlGUI);
        catch e
            if strcmp(e.message,'H must be the handle to a figure or figure descendent.')
                return
            end
        end
        
        % Set current elapsed time
        if G_flagTime
            set(handles.txt_time,'String',formatcell(G_Time,2));
        end
       
        % Set control mode visual
        if G_flagControlmode
            set(handles.txt_mode,'String',G_Controlmode);
        end
        
        % Set quadrotor setpoint table
        if G_flagnewQuadrotorSetpoint
            quadSetpoint = formatcell(G_QuadrotorSetpoint,2);
            % Update current setpoints table
            set(handles.tab_curset,'Data',quadSetpoint);
        end
        
        % Set quadrotor visual 
        if (G_flagQuadrotorState && G_flagPayloadPos)  
            
            %%% Update visual of Quadrotor with Payload
            % If not in MPC mode, the last MPC plan will be plotted
            for iSys = 1:model.nSys
                 eulers = zeros(3,model.nQuad);
                 for iQuad = 1:model.nQuad
                    eulers(:,iQuad)=G_QuadrotorState(cfg.Graphic.nQuadStates-3*(model.nQuad)+3*(iQuad-1)+(1:3), iSys);
                 end
                 VisualQuad(iSys).setPose(G_QuadrotorState(index.States.l, iSys),G_QuadrotorState(index.States.angles, iSys),eulers, G_MPCPlan(:, :, iSys));           
            end
            
            %%% Update table of Quadrotor
            % Update current inputs table with pitch, roll in degrees
            
            quadInputs = G_QuadrotorState(index.inputs,:);
            quadInputs(index.veldInputs,:) = rad2deg(quadInputs(index.veldInputs,:));
            quadInputs = reshape(quadInputs,3,length(quadInputs)/3)';
            quadInputs = formatcell(quadInputs,2);
            set(handles.tab_quadin,'Data',quadInputs);
            
            
            % Update current states table
            
            quadStates = G_QuadrotorState([index.States.l index.States.angles(:)' index.States.ld index.States.diff_angles(:)'],:).'; 
            quadStates = formatcell(quadStates,2);
            set(handles.tab_quadcur,'Data',quadStates);
            
            
            % Update current euler table
            
            %TODO: update this for muavp
            quadEuler = rad2deg(reshape(G_QuadrotorState(index.States.ss(end)+1:end),3,nQuad)');
            quadEuler = formatcell(quadEuler,1);
            set(handles.tab_quadeuler,'Data',quadEuler);
            
            
        end
        
        % Set obstacle visual
        if (G_flagObstacleState)
            
            %%% Update visual of Obstacle and if show subplots, the subplots 
            for iObs = 1:length(VisualObs)
                VisualObs(iObs).setPos(G_ObstacleState(:, iObs));
            end
            
            %%% Update table of obstacle
            G_ObstacleState = formatcell(G_ObstacleState,2);
            
        end
        
        % Set setpoint visual
        set(vis_set,'XData',setpointsQuadrotor(1,1),'YData',setpointsQuadrotor(2,1),'ZData',setpointsQuadrotor(3,1));
        
        drawnow limitrate
        
        %% Automated setpoints if toggled true
        if G_flagTime && handles.UserData.AutoSet
            setpointsQuadrotor = pr.fcnset(G_Time);
            GraphicCom.setQuadrotorSetpoint(setpointsQuadrotor);
        end
        
        %% Obtain new setpoints and simreset from GUI 
        
        flagNewHandles = 0;
        
        % Check if new Quadrotor setpoints have been set and triggered and
        % only publish if new
        if handles.UserData.flagNewSetpoints
            setpointsQuadrotor = handles.UserData.setpointsQuadrotor;
            % Send data
            GraphicCom.setQuadrotorSetpoint(setpointsQuadrotor);
            % Reset flag for GUI 
            handles.UserData.flagNewSetpoints = 0;

            flagNewHandles = 1;
        end
        
        % Check if a sim reset has been triggered and only publish if
        % there is a reset
        if handles.UserData.flagSimReset
            simresetObstacle = true;
            % Send data
            GraphicCom.setObstacleSimReset(simresetObstacle);
            % Reset flag for GUI
            handles.UserData.flagSimReset = 0;
            flagNewHandles = 1;
        end       
        
        % Send back handles with reset flag(s)
        if flagNewHandles
            guidata(ControlGUI, handles);
        end
        
    end
    
end
