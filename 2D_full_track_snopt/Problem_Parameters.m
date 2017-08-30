function Problem_Parameters()
addpath([pwd,'\NLP_Solver'],'-end')
addpath([pwd,'\SystemDynamics_and_TrackData'],'-end')

clear;clc;close all;

%physical parameters
r_cwf=1.2;
r_cwr=1.3;
mass=1;
grav=9.81;
mu_fric=1;
max_accel=mu_fric*mass*grav;
width=25;

%initial track position
X0=0;%x(1)
Y0=0;%x(2)
vX0=50;%x(4)
vY0=0;%x(5)
IC=[X0 Y0 vX0 vY0];%init. cond.

%simulation time
nSteps=40;
dt=.52;
time=dt*nSteps;
tspan=linspace(0,time,nSteps+1);

%System parameters
nStates=4;
nControls=2;

%track total arclength
%track_length=100+20*3*pi/4+50.7107+20*pi+50.7107+20*3*pi/4;
track_length=1000;

%resolution for rrt
control_res=13;

%initialization for direct transcription 
initial_guess=1;%0:zero 1:determinsitic 2:rrt

global_data = [nStates nSteps nControls dt max_accel mass grav mu_fric width control_res initial_guess IC];

save('Data_Bucket/params.mat')
end