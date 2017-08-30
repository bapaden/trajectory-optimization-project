%control force applied to dynamical equations

function f = u_init_1(x,y,xd,yd)

%use global variables
global global_data
nStates=global_data(1);
nSteps=global_data(2);
nControls=global_data(3);
dt=global_data(4);
max_accel=global_data(5);
mass=global_data(6);
grav=global_data(7);
mu_fric=global_data(8);
width=global_data(9);
control_res=global_data(10);
initial_guess=global_data(11);
IC=global_data(12:end);


kv=mass/2;
kp=mass/6;
kd=mass/2;
xdr=1/(k(x)+.025);

%max constant trim velocity is sqrt(g/kappa_max)
trim=mass*k(x)*(xd)^2;
f=[kv*(xdr-xd);-kp*y-kd*yd+trim];
if norm(f)/(mass*grav)>mu_fric
    f=mu_fric*(mass*grav)*f/norm(f);
    disp('traction loss')
end
f=f(1);
end