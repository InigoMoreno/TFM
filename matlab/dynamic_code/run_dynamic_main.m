init


x0i=model.lb+(model.ub-model.lb)/2;
x0i(isnan(x0i))=0;
x0=repmat(x0i',model.N,1);
problem.x0=x0; 

% Set initial and final conditions. This is usually changing from problem
% instance to problem instance:
problem.xinit = [2 1 pi/2 0]';
%problem.xfinal = model.xfinal;

% Set parameters
u=symunit;

L=to_units(16.5/2*u.in,u.m);
l=to_units(16.5/2*u.in,u.m); % not real, should be same as L
r=to_units((6/2-.75/2)*u.in,u.m);

problem.all_parameters = repmat([r L+l]',model.N,1);

[output,exitflag,info] = CynematicSolver(problem);

TEMP = zeros(model.nvar,model.N);
for i=1:model.N
    TEMP(:,i) = output.(['x',sprintf(['%0' num2str(length(num2str(model.N))) 'd'],i)]);
end
U = TEMP(index.u,:);
X = TEMP(index.x,:);
P=reshape(problem.all_parameters,2,[]);
Xdot=splitapply(@continuous_cynematics,X,U,P,1:model.N);

figure();
hold on
axis equal

line(X(1,:),X(2,:))

Rot =@(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];
square_raw=[L L -L -L L;...
            l -l -l l l];
for i=1:model.N
    x=X(1,i);
    y=X(2,i);
    th=X(3,i);
    square=[x;y]+Rot(th)*square_raw;
    line(square(1,:),square(2,:));
end