function x3dot=f3(x1,x2,x3,x4,u1,u2)

%forward_accel is the car frame forward acceleration
forward_accel=u1;

%lateral_accel is the car frame lateral acceleration
lateral_accel=u2;

%x3 is the downtrack velocity.
%The heading is fixed to be aligned with coordinate system
x3dot=(-1).*x3.*(1+(-1).*x2.*k(x1)).^(-1).*((-1).*x3.*x2.*dk(x1)+(-1).*x4.* ...
k(x1))+(1+(-1).*x2.*k(x1)).^(-1).*(forward_accel+x3.*x4.*k(x1));

end
