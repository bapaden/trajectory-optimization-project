function Execute_Trajectory_Optimization()

%% Clear workspace and setup paths
clear;clc;format compact;

addpath([pwd,'\NLP_Solver'],'-end')
addpath([pwd,'\SystemDynamics_and_TrackData'],'-end')

snscreen off
%% problem parameters
global global_data
Problem_Parameters();
load('Data_Bucket/params');

%% Setup initial guess

%Initial guess is zero
if initial_guess==0
    u0=zeros(nSteps,2);
    x0=zeros(nSteps+1,nStates);
    
    x0(1,:)=IC;
end

%Initial guess determined by conservative control
if initial_guess==1
    load('Data_Bucket/init_traj_deterministic')
    load('Data_Bucket/init_control_deterministic')
    u0=U;
    x0=X;
end

%Initial guess determined rrt
if initial_guess==2
    load('Data_Bucket/init_traj_rrt')
    load('Data_Bucket/init_control_rrt')
    u0=U;
    x0=X;
end
    
    
    
    %% format initial guess into decision variable
    u0=u0';
    y0=x0(2:end,:)';%throw out IC
    y0=[y0(:);u0(:)];
    
    %evaluate initial guess
    [F0,G0]=Objective_And_Constraint(y0);
    
    %% Optimization
    tic
    [yf,F,info] = Pass_Problem_To_SNOPT(nStates,nSteps,nControls,y0);
    toc
    [Ff,Gf]=Objective_And_Constraint(yf);
    %% Examine Optimization Results
    
    %reformat decision variable into trajectory and control
    xf=x0(1,:);
    for i=1:nSteps
        xf(i+1,:)=yf(nStates*(i-1)+1:nStates*i);
        uf(i,:)=yf(nSteps*nStates+nControls*(i-1)+1:nSteps*nStates+nControls*i);
    end
    
    
    
    %% Outputs to command window
    disp(' ')
    disp('-----Initial Performance-----')
    disp('The initial trajectory is:')
    disp(x0)
    disp('With control:')
    disp(u0')
    disp('Cost of initial guess')
    disp(F0(1))
    disp('---The decision variable is---')
    disp('State Values:')
    disp(y0(1:nStates*nSteps))
    disp('Control Values')
    disp(y0(nStates*nSteps+1:end))
    disp('The dynamic feasibility is (all should be zero):')
    disp(F0(2:nStates*nSteps+1));
    disp('The control contstraint (all should be negative):')
    disp(F0(nStates*nSteps+2:nStates*nSteps+nSteps+1));
    disp('The state constraint (all should be negative)');
    disp(F0(nStates*nSteps+nSteps+2:nStates*nSteps+nSteps+1+2*nSteps));

    
    disp(' ')
    disp('-----Final Performance-----')
    disp('The final trajectory is:')
    disp(xf)
    disp('With control:')
    disp(uf)
    disp('The final cost is:')
    disp(Ff(1))
    disp('The dynamic feasibility is (all should be zero):')
    disp(Ff(2:nStates*nSteps+1));
    disp('The control contstraint (all should be negative):')
    disp(Ff(nStates*nSteps+2:nStates*nSteps+nSteps+1));
    disp('The state constraint (all should be negative)');
    disp(Ff(nStates*nSteps+nSteps+2:nStates*nSteps+nSteps+1+2*nSteps));

    
    %% Save Results
    
    
    %     %initial trajectory
    %     X=x0;U=u0;
    %     save('Data_Bucket/init_traj','X');
    %     save('Data_Bucket/init_control','U');
    
    %final trajectory
    X=xf;
    U=uf;
    save('Data_Bucket/optimization_traj','X');
    save('Data_Bucket/optimization_control','U');
    Analyze_Results('optimization_result')
    
    
    
end