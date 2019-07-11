clear all

format shortG   

global model
%% Time declaration
model.time_lookup=1;
model.frequency=10; %Desired frequency in Hz
model.nNodes=6;
overtime=1;
integrator='ERK4';

%% Problem dimensions
model.N = ceil(model.time_lookup*model.frequency);   % length of multistage problem 
model.nin = 4; % number of inputs
model.nslack = 1; %number of slack variables


global index
index.u=1:model.nin;
index.slack=fold_struct(@max,index)+(1:model.nslack);
index.x.qr=1:3;
index.x.qr_dot=fold_struct(@max,index.x)+(1:3);


model.nstate=fold_struct(@max,index.x);
index.x.fromz=(1:model.nstate) +(model.nin+model.nslack);

model.nvar = model.nin+model.nstate+model.nslack; % number of stage variables 
model.neq = model.nstate;  % number of equality constraints 

set_static_parameters

model.nobs=4;

index.p.objective=1:3;
index.p.xlim=max(fold_struct(@max,index.p))+(1:2);
index.p.ylim=max(fold_struct(@max,index.p))+(1:2);

for i = 1:model.nobs
    index.p.obs_pos{i}=max(fold_struct(@max,index.p))+(1:2);
    index.p.obs_dim{i}=max(fold_struct(@max,index.p))+(1:2);
end

index.p.weight.position=max(fold_struct(@max,index.p))+1;
index.p.weight.angle=max(fold_struct(@max,index.p))+1;
index.p.weight.energy=max(fold_struct(@max,index.p))+1;
index.p.weight.slack=max(fold_struct(@max,index.p))+1;
index.p.weight.use_begin=max(fold_struct(@max,index.p))+1;
index.p.weight.obs_margin=max(fold_struct(@max,index.p))+1;


index.p.weights=fold_struct(@(x,y)[x y],index.p.weight);


model.npar = max(fold_struct(@max,index.p)); % number of runtime parameters 

for fields = fieldnames(index.x)'
    field=fields{1};
    if ~strcmp(field,'fromz')
        index.(field)=index.x.(field)+(model.nin+model.nslack);
    end
end

%% Objective function 

for i=1:model.N
    model.objective{i} = @(z,p) objective_dynamic(z,p)+p(index.p.weight.use_begin)*objective_dynamic_n(z,p) ;
end
model.objective{model.N} = @(z,p) objective_dynamic_n(z,p);


%% Dynamics, i.e. equality constraints 
model.continuous_dynamics = @(x,u,p)continuous_dynamics(x,u,p);
model.E = [zeros(model.nstate,model.nin+model.nslack), eye(model.nstate)];

%% Inequality constraints
% upper/lower variable bounds lb <= x <= ub
%            inputs    |        states
%             th1 th2 th3 th4  x y th e
model.lb = [ -24*ones(1,model.nin), zeros(1,model.nslack),  -inf*ones(1,model.nstate)  ];
model.ub = [ +24*ones(1,model.nin), +inf*ones(1,model.nslack), +inf*ones(1,model.nstate)  ];

model.xinitidx = index.x.fromz;

model.ineq=@(z,p)ineqs_dynamic(z,p);

[model.hl, model.hu] = ineqs_dynamic([],[]);

model.nh=length(model.hl);

% This is not used by forces, but it is good to have it in order to keep
% going if we have high equality error
switch integrator
    case 'ERK4'
        model.discrete_eq = @(z,p) RK4(z(index.x.fromz),z(1:model.nin),@continuous_dynamics,1/model.frequency,[],model.nNodes);
    case 'ERK3'
        model.discrete_eq = @(z,p) RK3(z(index.x.fromz),z(1:model.nin),@continuous_dynamics,1/model.frequency,[],model.nNodes);
    case 'ERK2'
        model.discrete_eq = @(z,p) RK2(z(index.x.fromz),z(1:model.nin),@continuous_dynamics,1/model.frequency,[],model.nNodes);
    otherwise
        model.discrete_eq = @(z,p) RK4(z(index.x.fromz),z(1:model.nin),@continuous_dynamics,1/model.frequency,[],model.nNodes);
end
codeoptions = getOptions('DynamicSolver');
codeoptions.nlp.integrator.type = integrator; 
codeoptions.nlp.integrator.Ts = 1/model.frequency;
codeoptions.nlp.integrator.nodes = model.nNodes;

codeoptions.maxit = floor(overtime/(model.frequency*model.N*1e-5));    % Maximum number of iterations
codeoptions.printlevel = 0; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 0;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed

codeoptions.overwrite = 1;
codeoptions.BuildSimulinkBlock = 0;
codeoptions.noVariableElimination = 1; 

filepath=fileparts(mfilename('fullpath'));
disp(filepath);
