function x4dot=f4(x1,x2,x3,x4,u1,u2)

%forward_accel is the car frame forward acceleration
forward_accel=u1;

%lateral_accel is the car frame lateral acceleration
lateral_accel=u2;

%x3 is the downtrack velocity.
%The heading is fixed to be aligned with coordinate system
x4dot=lateral_accel+(-1).*x3.^2.*k(x1).*(1+(-1).*x2.*k(x1));

end