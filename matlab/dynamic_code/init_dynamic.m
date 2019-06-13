clear all

%% Time declaration
model.time_lookup=1;
model.frequency=20; %Desired frequency in Hz

%% Problem dimensions
model.N = ceil(model.time_lookup*model.frequency);   % length of multistage problem 
model.nin = 4; % number of inputs
model.nh = 0;   % number of nonlinear inequality constraints 

global index
index.u=1:model.nin;
index.x.qr=1:3;
index.x.qr_dot=fold_struct(@max,index.x)+(1:3);

model.nstate=fold_struct(@max,index.x);
index.x.fromz=(1:model.nstate) +index.u(end);
model.nvar = model.nin+model.nstate; % number of stage variables 
model.neq = model.nstate;  % number of equality constraints 

set_static_parameters


index.p.objective=1:3;
index.p.xlim=fold_struct(@max,index.p)+(1:2);
index.p.ylim=fold_struct(@max,index.p)+(1:2);
index.p.weight.distance=fold_struct(@max,index.p)+1;
index.p.weight.energy=fold_struct(@max,index.p)+1;
index.p.weights=fold_struct(@(x,y)[x y],index.p.weight);

model.npar = fold_struct(@max,index.p); % number of runtime parameters 

for fields = fieldnames(index.x)'
    field=fields{1};
    if ~strcmp(field,'fromz')
        index.(field)=index.x.(field)+index.u(end);
    end
end

%% Objective function 

for i=1:model.N
    model.objective{i} = @(z,p) objective_dynamic(z,p) ;
end
model.objective{model.N} = @(z,p) objective_dynamic_n(z,p);
%model.objective{round(model.N*0.12)} = @(z,p) objective_dynamic_n(z,p);


%% Dynamics, i.e. equality constraints 
model.continuous_dynamics = @(x,u,p)continuous_dynamics(x,u,p);
model.E = [zeros(model.nstate,model.nin), eye(model.nstate)];

%% Inequality constraints
% upper/lower variable bounds lb <= x <= ub
%            inputs    |        states
%             th1 th2 th3 th4  x y th e
model.lb = [ -24*pi*ones(1,model.nin),  -inf*ones(1,model.nstate)  ];
model.ub = [ +24*pi*ones(1,model.nin),  +inf*ones(1,model.nstate)  ];

model.xinitidx = index.x.fromz;

model.ineq=@(z,p)[];

model.hu = []';
model.hl = []';



codeoptions = getOptions('DynamicSolver');
codeoptions.nlp.integrator.type = 'ERK2'; 
codeoptions.nlp.integrator.Ts = 1/model.frequency;
codeoptions.nlp.integrator.nodes = 5;

codeoptions.maxit = floor(1/(model.frequency*model.N*1e-5));    % Maximum number of iterations
codeoptions.printlevel = 0; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 0;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed

codeoptions.overwrite = 1;
codeoptions.BuildSimulinkBlock = 0;

filepath=fileparts(mfilename('fullpath'));
disp(filepath);
