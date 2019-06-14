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

plan_pub = rospublisher('/Plan','std_msgs/Float64MultiArray','IsLatching',false);
plan_msg = rosmessage(plan_pub);

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
X=[];
us=[];

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
        z0i(:)=0;
        z0i(index.x.fromz)=x;
        z0=repmat(z0i',model.N,1);
        problem.x0=z0; 

        reseted=false;
    end
    if run
        problem.xinit=x;
        problem.all_parameters = p;

        [output,exitflag,info] = DynamicSolver(problem);
        switch exitflag
            case 1
                disp('Local optimal solution found');
            case 0
                disp('Maximum number of iterations reached');
                %fprintf('Equality error: %f\n',info.res_eq);
            case -4
                warning('%i: Wrong number of inequalities input to solver',exitflag);
            case -5
                warning('%i: Error occured during matrix factorization.',exitflag)
            case -6
                warning('%i: NaN or INF occured during functions evaluations.',exitflag)
            case -7
                warning('%i: The solver could not proceed. Most likely cause is that the problem is infeasible.Try formulating a problem with slack variables (soft constraints) to avoid this error.',exitflag)
            case -10
                warning('%i: NaN or INF occured during evaluation of functions and derivatives.',exitflag)
            case -11
                warning('%i: Invalid values in problem parameters.',exitflag)
            case -100
                warning('%i: License error.',exitflag)
            otherwise
                warning('Exit flag is unknown: %i',exitflag)
        end
        %fprintf('Time per it per N: %f e-5\n',info.solvetime/(info.it*N)*1e5);
        Z = zeros(model.nvar,model.N);
        for i=1:model.N
            Z(:,i) = output.(['x',sprintf(['%0' num2str(length(num2str(model.N))) 'd'],i)]);
        end
        U = Z(index.u,:);
        X = Z(index.x.fromz,:);
        P=reshape(problem.all_parameters,model.npar,[]);
        Xdot=splitapply(@continuous_dynamics,X,U,P,1:model.N);
        if info.res_eq>1e-4
            warning('High equality error');
            x=model.discrete_eq(Z(:,1),P(:,1));
        else
            x=X(:,2);
        end
        us=[us U(:,1)];
        x0tmp=Z(:,[2:end end]);%use last solution
        x0tmp=x0tmp+ 0.05*rand(size(x0tmp)); % Add some random variations
        problem.x0=reshape(x0tmp,[],1);
    end
    if ~reseted
        state_msg.Data=x;
        state_pub.send(state_msg);
        if ~isempty(X)
            plan_msg.Data=reshape(X(index.x.qr,:),[],1);
            plan_pub.send(plan_msg);
        end
    end
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