clear

syms l L r

spi=sym(pi);
alpha=[1 -1 -1 1]*spi/4;
beta=[0 0 0 0];
s=[1 1 -1 -1]*L;
d=[1 -1 1 -1]*l;

R=sym(zeros(4,3));

for i=1:4
    R(i,1)=sin(alpha(i)+beta(i))/(r*sin(alpha(i)));
    R(i,2)=-cos(alpha(i)+beta(i))/(r*sin(alpha(i)));
    R(i,3)=(-d(i)*sin(alpha(i)+beta(i))-s(i)*cos(alpha(i)+beta(i)))/(r*sin(alpha(i)));
end
R=simplify(R);
disp(R*r)
