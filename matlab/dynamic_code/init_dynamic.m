clear

%% Problem dimensions
model.N = 100;   % length of multistage problem 
model.nin = 4; % number of inputs
model.nstate = 4; % number of state variables
model.nvar = model.nin+model.nstate; % number of stage variables 
model.neq = 4;  % number of equality constraints 
model.nh = 0;   % number of nonlinear inequality constraints 
model.npar = 2; % number of runtime parameters 

index.u=1:model.nin;
index.x=(1:model.nstate) +index.u(end);

%% Objective function 
for i=1:model.N-1
    model.objective{i} = @(z) 0.2*z(1)^2 + 0.01*z(2)^2 + 0.01*z(3)^2 + 0.01*z(4)^2;
end
model.objective{model.N} = @(z) 100*z(index.x(1))^2 + 100*z(index.x(2))^2 + 100*z(index.x(3))^2 + 100*z(index.x(4))^2;

%% Dynamics, i.e. equality constraints 
model.continuous_dynamics = @(x,u,p)continuous_cynematics(x,u,p);
model.E = [zeros(model.nstate,model.nin), eye(model.nstate)];

%% Inequality constraints
% upper/lower variable bounds lb <= x <= ub
%            inputs    |        states
%             th1 th2 th3 th4  x y th e
model.lb = [ -2.4*2*pi*ones(1,4),  -inf*ones(1,4)  ];
model.ub = [ +2.4*2*pi*ones(1,4),  +inf*ones(1,4)  ];

model.xinitidx = index.x;

model.ineq=@(z)[];

model.hu = []';
model.hl = []';



codeoptions = getOptions('CynematicSolver');
codeoptions.nlp.integrator.type = 'ERK2'; 
codeoptions.nlp.integrator.Ts = 0.1;
codeoptions.nlp.integrator.nodes = 5;

codeoptions.maxit = 200;    % Maximum number of iterations
codeoptions.printlevel = 0; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 0;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed

codeoptions.overwrite = 1;
codeoptions.BuildSimulinkBlock = 0;

filepath=fileparts(mfilename('fullpath'));
disp(filepath);

%% Generate forces solver
get_new_forces=0;
forces_path=fullfile(filepath,codeoptions.name);
old_file=fullfile(forces_path,'old.mat');
if ~get_new_forces && exist(old_file,'file')
    load(old_file,'old');
    new.codeoptions=codeoptions;
    new.model=model;
    z_sym=sym('z',[new.model.nvar 1]);
    x_sym=sym('x',[new.model.nstate 1]);
    u_sym=sym('u',[new.model.nin 1]);
    p_sym=sym('p',[new.model.npar 1]);
    new.model.objective=cellfun(@(x)char(x(z_sym)),new.model.objective,'UniformOutput',0);
    new.model.continuous_dynamics=char(new.model.continuous_dynamics(x_sym,u_sym,p_sym));
    new.model.ineq=char(new.model.ineq(z_sym));
    if ~isequal(old.codeoptions,new.codeoptions) || ~ isequal(old.model,new.model)
        get_new_forces=1;
    end
else
    get_new_forces=1;
end

if get_new_forces
    if ~exist(forces_path,'dir')
        mkdir(forces_path)
    end
    old.model=model;
    old.codeoptions=codeoptions;
    z_sym=sym('z',[old.model.nvar 1]);
    x_sym=sym('x',[old.model.nstate 1]);
    u_sym=sym('u',[old.model.nin 1]);
    p_sym=sym('p',[old.model.npar 1]);
    old.model.objective=cellfun(@(x)char(x(z_sym)),old.model.objective,'UniformOutput',0);
    old.model.continuous_dynamics=char(old.model.continuous_dynamics(x_sym,u_sym,p_sym));
    old.model.ineq=char(old.model.ineq(z_sym));
%     if exist(forces_path,'dir')
%         rmdir(forces_path,'s');
%     end
%     mkdir(forces_path)
    x=cd(forces_path);
    try
        FORCES_NLP(model, codeoptions);
    catch e
        cd(x)
        rethrow(e)
    end
    save(old_file,'old');
    cd(x);
    addpath(genpath(forces_path))
else
    fprintf('Your code is already up to date, no need for compiling\n');
end