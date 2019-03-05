use_hendzel_syms=1;
test_syms
psis_dot=sym('psi_dot',[4 1]);
syms vsxp vsyp B_dot
X=[vsxp vsyp B_dot].';

%2.16--2.18
pretty(X==simplify(expand(Ri*psis_dot)))
X_rhs=simplify(expand(Ri*psis_dot));
X_rhs_sq=X_rhs.^2;

syms A B C
syms mpc Ipc Ik
Ik=C;
%A=mpc*r^2/8
%B=Ipc*r^2/(16*(L+l)^2)
X_rhs_sq(1:2)=subs(X_rhs_sq(1:2),r^2,8*A/mpc);
X_rhs_sq(3)=subs(X_rhs_sq(3),r^2/(L+l)^2,B*16/Ipc);




T=.5*mpc*(vsxp^2+vsyp^2)+.5*Ipc*B_dot^2+.5*Ik*(psis_dot.'*psis_dot);
%T=.5*Ik*(psis_dot.'*psis_dot);
T=subs(T,X.^2,X_rhs_sq);
T=simplify(expand(T));

dTdpsi_dot=gradient(T,psis_dot);
syms t psi_dot1(t) psi_dot2(t) psi_dot3(t) psi_dot4(t)
psis_dot_t=[psi_dot1(t) psi_dot2(t) psi_dot3(t) psi_dot4(t)].';
dTdpsi_dot_tdt=diff(subs(dTdpsi_dot,psis_dot,psis_dot_t),t);

psis_ddot=sym('psi_dot',[4 1]);
Qs=sym('Q',[4 1]);
dTdpsi_dot_dt=subs(dTdpsi_dot_tdt,diff(psis_dot_t,t),psis_ddot);
M=equationsToMatrix(dTdpsi_dot_dt==Qs,psis_ddot);




