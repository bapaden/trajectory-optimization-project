function deterministic_trajectory_initialization()
%curved coordinate system simulation. Used to create initial guess.
clear;clc;close all;
Problem_Parameters()

global global_data
load('Data_Bucket/params')


% Fzf=m*g*r_cwr/(r_cwr+r_cwf);%update for load transfer
% Fzr=m*g*r_cwf/(r_cwr+r_cwf);%update for load transfer


%simulation parameters
state_0=IC';

%dynamics
sys=@(x) [f1(x(1),x(2),x(3),x(4),u_init_1(x(1),x(2),x(3),x(4)),u_init_2(x(1),x(2),x(3),x(4)));
          f2(x(1),x(2),x(3),x(4),u_init_1(x(1),x(2),x(3),x(4)),u_init_2(x(1),x(2),x(3),x(4)));
          f3(x(1),x(2),x(3),x(4),u_init_1(x(1),x(2),x(3),x(4)),u_init_2(x(1),x(2),x(3),x(4)));
          f4(x(1),x(2),x(3),x(4),u_init_1(x(1),x(2),x(3),x(4)),u_init_2(x(1),x(2),x(3),x(4)))];
X=zeros(nSteps,4);
X(1,:)=IC';

f4(state_0(1),state_0(2),state_0(3),state_0(4),u_init_1(state_0(1),state_0(2),state_0(3),state_0(4)),u_init_2(state_0(1),state_0(2),state_0(3),state_0(4)));
for i=1:nSteps
    X(i+1,:)=X(i,:)+dt*sys(X(i,:))';
    U(i,:)=[u_init_1(X(i,1),X(i,2),X(i,3),X(i,4)) u_init_2(X(i,1),X(i,2),X(i,3),X(i,4))]';
end

   
      

save('Data_Bucket/init_traj_deterministic','X')
save('Data_Bucket/init_control_deterministic','U')
Analyze_Results('initial_guess_determinsitic')


end
