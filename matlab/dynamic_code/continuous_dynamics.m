function x_dot = continuous_dynamics(x, u, p)
         global index static_parameters
         r=static_parameters.r;
         Ll=static_parameters.L+static_parameters.l;
         m=static_parameters.m;
         Iz=static_parameters.Iz;
         Iw=static_parameters.Iw;
         Km=static_parameters.K;
         Rm=static_parameters.R;
         N=static_parameters.N;
         nu=static_parameters.nu;
         a=static_parameters.a;
         b=static_parameters.b;
         
         V=u;
         
         qr=x(index.x.qr);
         qr_dot=x(index.x.qr_dot);
         
         H=diag([m m Iz])+(4*Iw/r^2)*diag([1 1 Ll^2]);
         K=(4*Iw/r^2)*qr_dot(3)*[0 1 0;-1 0 0;0 0 0];
         R=(1/r)*[1 -1 -Ll; 1 1 Ll; 1 1 -Ll; 1 -1 Ll];
         Rot =[cos(qr(3)) -sin(qr(3)) 0; sin(qr(3)) cos(qr(3)) 0; 0 0 1];
         qw_dot = R*transpose(Rot)*qr_dot;
         
         %DC motor
         Tw=nu*N*Km*(V-N*Km*qw_dot)/Rm;
         
         %Estimated friction
         Tfric=a.*qw_dot+b.*sign(qw_dot);
         
         T=Tw-Tfric;
         
         Fa=(1/r)*[sin(qr(3))*(T(1)-T(2)-T(3)+T(4))+cos(qr(3))*(T(1)+T(2)+T(3)+T(4));
                   sin(qr(3))*(T(1)+T(2)+T(3)+T(4))-cos(qr(3))*(T(1)-T(2)-T(3)+T(4));
                           -Ll*(T(1)-T(2)+T(3)-T(4)) ];
         
         qr_ddot=inv(H)*(Fa-K*qr_dot);
                       
         x_dot=zeros(size(x),'like',x);
         
         x_dot(index.x.qr)=qr_dot;
         x_dot(index.x.qr_dot)=qr_ddot;
         
end

