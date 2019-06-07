function x_dot = continuous_dynamics(x, u, p)
         global index
         r=p(index.p.r);
         L=p(index.p.L);
         A=p(index.p.A);
         B=p(index.p.B);
         C=p(index.p.C);
         K=p(index.p.K);
         R=p(index.p.R);
         a=p(index.p.a);
         b=p(index.p.b);
         
         qr=x(index.x.qr);
         qw_dot=x(index.x.qw_dot);
         
         V=u;
         
         Rei=[ 1,   1,    1,   1  ;...
              -1,   1,    1,  -1  ;...
              -1/L, 1/L, -1/L, 1/L;...
               1,   1,   -1,  -1  ]*(r/4);
           
         M= [ A + B + C,        -B,         B,     A - B;...
                     -B, A + B + C,     A - B,         B;...
                      B,     A - B, A + B + C,        -B;...
                  A - B,         B,        -B, A + B + C];
          
         %From agullo
         qr_dot_rel=Rei*qw_dot;
         
         %Rotate reference
         Rot =@(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];
         qr_dot=[Rot(x(3))*qr_dot_rel(1:2);qr_dot_rel(3);abs(qr_dot_rel(4))^2];
         
         %DC motor
         Tw=(V*K-qw_dot)/R;
         
         %Estimated friction
         Tfric=a*qw_dot+b*sign(qw_dot);
         
         %From hendzel
         qw_ddot=inv(M)*(Tw-Tfric); %needs rotation
         
         x_dot=zeros(size(x),'like',x);
         
         x_dot(index.x.qr)=qr_dot;
         x_dot(index.x.qw_dot)=qw_ddot;
         
end

