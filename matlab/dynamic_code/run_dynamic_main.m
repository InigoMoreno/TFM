init_dynamic


x0i=model.lb+(model.ub-model.lb)/2;
x0i=x0i(index.x.fromz);
x0i(isnan(x0i))=0;
x0=repmat(x0i',model.N,1);
problem.x0=x0; 

% Set initial and final conditions. This is usually changing from problem
% instance to problem instance:
problem.xinit = [2 1 pi/2 0 0 0 0 0]';
%problem.xfinal = model.xfinal;

% Set parameters
u=symunit;

keep_units=0;

L=to_si(16.5/2*u.in,keep_units);
l=to_si(16.5/2*u.in,keep_units); % not real, should be same as L
r=to_si((6/2-.75/2)*u.in,keep_units);
m=to_si(16*u.kg-250*u.g,keep_units);
%Iz=to_si(11.61*u.kg*u.m^2,keep_units);
Iz=to_si(20*u.kg*u.m^2,keep_units);
Ik=to_si(1/2*2.3*u.kg*r^2,keep_units);


Ke=to_si(24*u.V/(7000*u.rpm),keep_units);
Km=to_si(4250*u.g*u.cm  *  9.81*u.m/u.s^2 / (13*u.A),keep_units);
R=to_si(24*u.V/(13*u.A),keep_units);

A=m*r^2/8;
B=Iz*r^2/(16*(L+l)^2);
C=Ik;



p=zeros(model.npar,1);
p(index.p.r)=r;
p(index.p.L)=L+l;

p(index.p.A)=A;
p(index.p.B)=B;
p(index.p.C)=C;

p(index.p.K)=Ke; %Ke=Km, we use Ke as it is more direct from the datasheet.
p(index.p.R)=R;
%NEED TO SET OTHER PARAMS

problem.all_parameters = repmat(p,model.N,1);

[output,exitflag,info] = CynematicSolver(problem);

TEMP = zeros(model.nvar,model.N);
for i=1:model.N
    TEMP(:,i) = output.(['x',sprintf(['%0' num2str(length(num2str(model.N))) 'd'],i)]);
end
U = TEMP(index.u,:);
X = TEMP(index.x.fromz,:);
P=reshape(problem.all_parameters,model.npar,[]);
Xdot=splitapply(@continuous_dynamics,X,U,P,1:model.N);

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