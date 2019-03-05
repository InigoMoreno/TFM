%% Initalise script 
% Contains all index definitions, system characteristics 

% Vector z is the stage variable vector
% Vector z = [ 1   2    3     4    5    6     7     8        9           10 11 12 13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28    29    30    31    32    33    34    35    36    37    38    39]
% Vector z = [thu1 phu1 gazu1 thu2 phu2 gazu2 slack slackenv slackenvneg xl yl zl xq1 yq1 zq1 xq2 yq2 zq2 xld yld zld xw1 yw1 zw1 xw2 yw2 zw2 xth11 xth21 xph11 xph21 xzd11 xzd21 xth12 xth22 xph12 xph22 xzd12 xzd22]
% First 6 entries are inputs, next 2 are slacks, next 18 entries are the UAVP state, last 12 the UAV dyanamics state space states

% Global pr required for the MPC_uavpdynamics
global  pr index model

%% Add paths for FORCES and CasADi
addpath(genpath(fullfile(fileparts(fileparts(fileparts(pwd))), 'FORCES_PRO')))
addpath(genpath(fullfile(fileparts(fileparts(fileparts(pwd))), 'casadi-matlabR2014b-v2.4.2')))
addpath(pwd, 'mpc')
addpath(pwd, 'dynamics')
addpath(pwd, 'data')  
addpath(pwd, 'estimator')
addpath(pwd, 'functions')
addpath(pwd, 'scenario')

%% Scenario load

% Load scenario file. USER CHANGEABLE
if ~exist('no_load_scene','var') || ~no_load_scene
    clear nObs nQuad
    scn_highpole2
else
    no_load_scene = 0;
end

% Set default quadrotor setpoints
setpointsQuadrotor = zeros(3,2,nSys);
for i = 1:nSys
    setpointsQuadrotor(:,:,i) = [loadStartPos(:,i), loadEndPos(:,i)];
end

%% MATLAB and Configuration Setup

cfg.verbose = true; % Display progress (iterations)
cfg.logsize = 30000; %based on sampling at around 0.02s for 10 minutes
cfg.DoLog = true;

%%% Set to experimental or simulation mode (Default experimental)
% Set obsSim = 1 for simulated obstacles!
% Set Simdt to time step used for simulation (affects real time factor).
% Set UseRealdt to use real dt for control loop even in simulation. (RT factor is always 1).
cfg.ModeSim = 1;
cfg.UseRealdtSim = 0;
cfg.externalPlanner = 0;
cfg.switch_and_stop = 0; %for automatic testing, switches x when objective is reached and stops at 20s
cfg.addnoise = 0;

if ~exist('cfg','var') || ~isfield(cfg,'dtSim')
    cfg.dtSim = 1/12;
end

if ~exist('model','var') || ~isfield(model,'N')
    model.N = ceil(1.2/cfg.dtSim);    %15       % horizon length
end

% Get new FORCES solver from web
cfg.get_new_forces = 0;


% Minimum dt (for experimental)
cfg.mindt = 0.005;

% Motion Capture System type (choose one for experimental)
cfg.UseOpti = ~cfg.ModeSim;
cfg.UseVicon = 0;

% Control form (bodycontrol allows control in body axes (DO NOT USE IN COMBINATION WITH NON-MANUAL CONTROLLERS))
cfg.UseBodycontrol = 0;

% Simulated obstacles
cfg.obsSim = 1;
if cfg.ModeSim; cfg.obsSim = 1; end %obsSim simulated if in simulation mode


% Define configuration for live plotting in GUI
cfg.Visual.ShowSubplot = 0;  % Only default value at startup, user changeable in GUI (this cfg is not changed)
cfg.Visual.ShowMPCPlan = 1;
cfg.Visual.ShowObsEll = 0;
cfg.Visual.ShowObsEncEll = 1;

%% MPC Model definition

%%% Problem dimensions

model.nSys = nSys;
model.nQuad = nQuad;
model.nParamsPerQuad = 3 + 2*nQuad; % number of parameters per system
model.nObs = nObs;       % number of obstacles static/dynamic
model.nParamsPerObs = 3; % number of parameters per obstacle static/dynamic (position only)
model.nSlack = 2; % number of slack variables in use (need to add after input in z)

model.nvar = 8+13*nQuad;        % number of variables in z
model.nin =3*nQuad; % number of inputs (slacks are not input)
model.neq = model.nvar - model.nin - model.nSlack;         % number of equality constraints
model.nh = 6+8*nQuad +nQuad*(nQuad-1)/2+ model.nObs;           % number of inequality constraint functions (3 for load, 3 for quad, 1 for wires, 1 for quadspeeds, n*(n-1)/2 for distance, 3 for payload, 1 inequalties per obstacle)
model.npar = 7 + (model.nParamsPerObs)*model.nObs + model.nParamsPerQuad*(model.nSys-1); % number of runtime parameters in p, 6 for start and end point, 1 for realdt, then nObs for obstacle detection state then rest for obstacle and quadrotor positions

model.nuavp_per_quad=6;
model.nuavp = model.nuavp_per_quad*nQuad;       % number of UAVP system states

% Visual data and variables for CGraphic
cfg.Graphic.nQuadStates = model.nvar+3*nQuad; % communicating all of Z_k (38) and euler angles (6)
cfg.Graphic.nQuadSetpoints = 3; % only communicating payload positions
cfg.Graphic.nQuadMPCPlan = 3+2*nQuad; % communicating Positions and Payload angles
cfg.Graphic.nObsStates = 6; % communicating position and velocity

%% Indexing

% Relates the quadID and obsID from MATLAB to the IDs set in ROS
mapping = [3,4,1,2];
mapping_obs = [1,2,3,4];

%%% z vector (stage vector) (make sure to increment following indices if entry is added)

index.inputs = 1:model.nin;
index.veldInputs = [];
index.velzInputs = [];
for i=1:nQuad
    index.input(i,:)=(i-1)*3+(1:3);
    index.veldInputs = [index.veldInputs index.input(i,1:2)];
    index.velzInputs = [index.velzInputs index.input(i,3)];
end

index.slackCollision = model.nin+1;
index.slackEnv = model.nin+2;

index.slacks = [index.slackCollision; index.slackEnv];

% Vector z = [ 1   2    3     4    5    6     7     8        9           10 11 12 13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28    29    30    31    32    33    34    35    36    37    38    39]
% Vector z = [thu1 phu1 gazu1 thu2 phu2 gazu2 slack slackenv slackenvneg xl yl zl xq1 yq1 zq1 xq2 yq2 zq2 xld yld zld xw1 yw1 zw1 xw2 yw2 zw2 xth11 xth21 xph11 xph21 xzd11 xzd21 xth12 xth22 xph12 xph22 xzd12 xzd22]
%                                                                        1  2  3  4   5   6   7   8   9   10  11  12  13  14  15   16  17  18 19    20    21    22    23    24    25    26    27    28    29    30

index.x.fromz = (model.nin+3):model.nvar;
index.x.States.l = 1:3; 
for i=1:nQuad
    index.x.States.angles(i,:)=index.x.States.l(end)+(i-1)*2+(1:2);
end
index.x.States.ld = index.x.States.angles(end)+(1:3);
for i=1:nQuad
    index.x.States.diff_angles(i,:)=index.x.States.ld(end)+(i-1)*2+(1:2);
end
for i=1:nQuad
    index.x.States.ss(i,:)=index.x.States.diff_angles(end)+(i-1)*6+(1:6);
end

for i = fieldnames(index.x.States)'
    index.States.(i{1})=index.x.States.(i{1})+index.slackEnv;
end
%%% p vector (real-time parameters)

index.startpointProblem = 1:3;
index.endpointProblem = 4:6;
index.realdt = 7;

% Obstacle parameter indices for p, rows are entries and each column is one obstacle
if model.nObs >= 1
    newidx = index.realdt + 1;
    index.obsParams = reshape(newidx:((newidx-1)+model.nParamsPerObs*model.nObs),[model.nParamsPerObs,model.nObs]);
    index.obsPos = 1:3; % This is the index within the obsParams indices

    % Obstacle parameter indices for all_parameters entries (all stages N) for
    % first obstacle. index.obsParamsN + 1 gives indices of next obstacle 
    idxObsPos = (index.obsParams(1)):model.npar:(index.obsParams(1)) + model.npar*model.N - 1;
    index.obsParamsN = reshape([idxObsPos; idxObsPos + 1; idxObsPos + 2],[model.nParamsPerObs*model.N, 1]);
end

% Quad parameter indices for p, rows are entries and each column is one
% quad
if model.nSys > 1
    if model.nObs >= 1
        obsParams_end = index.obsParams(end);
    else
        obsParams_end = index.realdt;
    end
    
    index.quadParams = reshape((obsParams_end+1):(obsParams_end +model.nParamsPerQuad*(model.nSys-1)),[model.nParamsPerQuad,(model.nSys-1)]);
    index.quad.l  = 1:3; % This is the index within the quadParams indices
    for i=1:nQuad
        index.quad.angles(i,:)=ifndex.quad.l(end)+2*(i-1)+(1:2);
    end

    % Quad parameter indices for all_parameters entries (all stages N) for
    idxQuadPos = (index.quadParams(1)):model.npar:(index.quadParams(1) + model.npar*model.N - 1);
    index.quadParamsN = repmat(idxQuadPos,model.nParamsPerQuad,1);
    index.quadParamsN = bsxfun(@plus,index.quadParamsN,(0:(model.nParamsPerQuad-1))');
    index.quadParamsN = reshape(index.quadParamsN,[model.nParamsPerQuad*model.N, 1]);
end

clear newidx idxObsPos idxQuadPos obsParams_end 

%% Characteristics of system and environment

%%% System characteristics 

% Gravitational acceleration
pr.g = 9.81;        %m/s^2

% Quadrotor and payload mass
pr.m = 0.5*ones(1,nQuad);          %kg
pr.ml = 0.0111;       %kg

% Suspension cable length
pr.l(1) = 0.8211;         %m
pr.l(2) = 0.7328;         %m
pr.l(3) = 0.77;         %m
pr.l(4) = 0.77;         %m

% Drag

pr.D.q = 0.28;
pr.D.angl = 0.5*1.47*1.225*pi*(0.025^2)*(pr.l(1)^3);
pr.D.l = 0.5*1.47*1.225*pi*(0.025^2); % Used for uavpdynamicsQD Quadratic drag

% *** System limits ***

% Quadrotor detection limits
pr.detect.radius = 3.5; %m
pr.detect.points = 18; % circular discretisation 
pr.detect.steerthreshold = 0.75;

% Input and state limits

pr.input.maxangle = deg2rad(15); 
pr.input.maxgaz = 2; %m/s

pr.state.mintension = -.05; %minimum tension of wires set to small negative value to prevent drones from overreacting s to avoid pushing into the ball.

pr.state.maxvel = 2;
pr.state.maxswing = deg2rad(85); % 60

% Workspace limits [lower upper]


if ~exist('pr','var') || ~isfield(pr,'ws')
    pr.ws.xdim = [-3 3]; %m [-2.8 2.8] [-3.0 3.0]
    pr.ws.ydim = [-1.5 1.5]; %m [-1.5 1.5]
    pr.ws.zdim = [0.1 2.6]; %m [-,2.6] Added 0.1cm to ground level so there is no scraping along the floor. [0.1 1.8]
end
pr.ws.dim = [pr.ws.xdim(2) - pr.ws.xdim(1), pr.ws.ydim(2) - pr.ws.ydim(1), pr.ws.zdim(2) - pr.ws.zdim(1)];

% Quadrotor and Payload radius constraints
pr.coll.qd = 0.3;
pr.coll.pl = 0.2;

%%% Attitude and Gaz dynamics 

% Load identified gaz acceleration, pitch and roll attitude stabilisation
% dynamics as state spaces
w=warning('off','all');
load('data/UAV_identifiedss')
load('data/Inigo_identifiedss')
warning(w)
clear w

pr.att.Ap = sys_pitch.A;
pr.att.Bp = sys_pitch.B;
pr.att.Cp = sys_pitch.C;
pr.att.Dp = sys_pitch.D;

pr.att.Ar = sys_roll.A;
pr.att.Br = sys_roll.B;
pr.att.Cr = sys_roll.C;
pr.att.Dr = sys_roll.D;

pr.att.Ag = sys_gazacc.A;
pr.att.Bg = sys_gazacc.B;
pr.att.Cg = sys_gazacc.C;
pr.att.Dg = sys_gazacc.D;

pr.att.Ag_d = ss5.A;
pr.att.Bg_d = ss5.B;
pr.att.Cg_d = ss5.C;
pr.att.Dg_d = ss5.D;

pr.att.limg2=(pr.att.Ag(1,1)*pr.att.Bg(2)-pr.att.Ag(2,1)*pr.att.Bg(1))/(-det(pr.att.Ag));
pr.att.klimg=-pr.att.Cg_d*(pr.att.Ag_d\pr.att.Bg_d);

clear ss2pem_gaz ss2pem_pitch ss2pem_roll sys_gaz sys_gazacc sys_pitch sys_roll ss5

%% Automated setpoint tracking

% Set any function of current time that returns a vector setpoint
pr.fcnset = @(curtime) fcn_setpoint(curtime);

%% Store all initialise variables in share
share.cfg = cfg;
share.model = model;
share.index = index;
share.pr = pr;
