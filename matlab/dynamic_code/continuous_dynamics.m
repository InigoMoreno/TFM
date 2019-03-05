function xdot = continuous_cynematics(x, u, p)
         r=p(1);
         L=p(2);
         Rei=[ 1,   1,    1,   1  ;...
              -1,   1,    1,  -1  ;...
              -1/L, 1/L, -1/L, 1/L;...
               1,   1,   -1,  -1  ]*(r/4);
         xdot_rel=Rei*u;
         
         Rot =@(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];
         xdot=[Rot(x(3))*xdot_rel(1:2);xdot_rel(3);abs(xdot_rel(4))^2];
end

