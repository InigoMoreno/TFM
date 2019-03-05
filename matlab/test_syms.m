clearvars -except a use_hendzel_syms

syms l L r

if exist('use_hendzel_syms','var') && use_hendzel_syms
    syms sx sy R
    r=R+r;
    l=sx;
    L=sy;
end

spi=sym(pi);
alpha=[1 -1 -1 1]*spi/4;
beta=[0 0 0 0];
s=[1 1 -1 -1]*L;
d=[1 -1 1 -1]*l;

%We generate the matrix R based on agullo's paper
R=sym(zeros(4,3));
for i=1:4
    R(i,1)=sin(alpha(i)+beta(i))/(r*sin(alpha(i)));
    R(i,2)=-cos(alpha(i)+beta(i))/(r*sin(alpha(i)));
    R(i,3)=(-d(i)*sin(alpha(i)+beta(i))-s(i)*cos(alpha(i)+beta(i)))/(r*sin(alpha(i)));
end
R=simplify(R);
Ri=pinv(R);

syms v1 v2 w
V=[v1;v2;w];
U=sym('u',[4,1]);

%U==R*V
%U(1:3)==R(1:3,:)*V
%inv(R(1:3,:))*U(1:3)==V
%R*inv(R(1:3,:))*U(1:3)==U
%We can reconstruct U from only its first three components
%Therefore we can find an equation that relates them
Ur=simplify(R*inv(R(1:3,:))*U(1:3));
A=equationsToMatrix(Ur(4)==U(4),U);

%We add a column to the R matrix, now V will have[v1;v2;w;e] were e is the
%unused velocity. We divide it by r so it is similar to other columns
Re=[R A.'/r];

%Now we can find an inverse! See that the first three rows of the inverse
%are the same as the pseudoinverse of R
Rei=inv(Re);

