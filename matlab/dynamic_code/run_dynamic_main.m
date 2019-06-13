close all
clear

init_dynamic
generate_forces
setupROS

z0i=model.lb+(model.ub-model.lb)/2;
z0i(isnan(z0i))=0;
z0=repmat(z0i',model.N,1);
problem.x0=z0; 

% Set initial and final conditions. This is usually changing from problem
% instance to problem instance:

% Set parameters
u=symunit;

keep_units=0;



state_pub = rospublisher('/State','std_msgs/Float64MultiArray','IsLatching',false);
state_msg = rosmessage(state_pub);

reset_sub     = rossubscriber('/Reset','std_msgs/Float64MultiArray',@resetListener);
params_sub = rossubscriber('/Parameters','std_msgs/Float64MultiArray',@paramsListener);
run_sub = rossubscriber('/Run','std_msgs/Bool',@runListener);

global p
global run
global problem
global reseted

problem.xinit=[];
p=[];
run=[];
reseted=false;

disp('Waiting for visual to be opened')

r=robotics.Rate(2);
while(isempty(problem.xinit) || isempty(p) || isempty(run))
    waitfor(r);
end
disp('Visual opened');
r=robotics.Rate(model.frequency);
while true
    if reseted
        x=problem.xinit;
        z0i=model.lb+(model.ub-model.lb)/2;
        z0i(isnan(z0i))=0;
        %z0i(index.x.fromz)=x;
        z0=repmat(z0i',model.N,1);
        problem.x0=z0; 

        reseted=false;
    end
    if run
        problem.xinit=x;
        problem.all_parameters = repmat(p,model.N,1);

        [output,exitflag,info] = DynamicSolver(problem);
        if exitflag == 0
            disp('MPC: max it');
        end
        if ~(exitflag == 1 || exitflag ==0)
            warning('Exit flag is %i',exitflag) 
        end
        disp(info.solvetime/(info.it*N));
        Z = zeros(model.nvar,model.N);
        for i=1:model.N
            Z(:,i) = output.(['x',sprintf(['%0' num2str(length(num2str(model.N))) 'd'],i)]);
        end
        U = Z(index.u,:);
        X = Z(index.x.fromz,:);
        P=reshape(problem.all_parameters,model.npar,[]);
        Xdot=splitapply(@continuous_dynamics,X,U,P,1:model.N);
        x=X(:,2);
        problem.x0=reshape(Z(:,[2:end end]),[],1);
    end
    state_msg.Data=x;
    state_pub.send(state_msg);
    drawnow limitrate
    waitfor(r);
end

function resetListener(~,reset_msg)
    global problem reseted
    problem.xinit=reset_msg.Data;
    reseted=true;
end
function paramsListener(~,params_msg)
    global p
    p=params_msg.Data;
end
function runListener(~,params_msg)
    global run
    run=params_msg.Data;
end