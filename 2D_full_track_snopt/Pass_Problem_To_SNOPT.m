 function [y,F,INFO] = Pass_Problem_To_SNOPT(nStates,nSteps,nControls,y0)
%function [x,F,INFO] = snoptmain2()
% Defines the NLP problem and calls the mex interface for snopt.
% First derivatives are provided.

%snoptmain2.spc = which('snoptmain2.spc');

snprint('Data_Bucket/Output_File.out');
% snspec ( snoptmain2.spc  );
%snseti ('Major Iteration limit', 300);

%Get condensed data for the problem setup.
[xlow,xupp,Flow,Fupp,A,iAfun,jAvar,iGfun,jGvar] = Setup_Problem();


% Optimization type
snset  ('Minimize');

[y,F,INFO] = snopt(y0,xlow,xupp,Flow,Fupp,'Objective_And_Constraint',A, iAfun, jAvar, iGfun, jGvar);

snprint off; % Closes the file and empties the print buffer

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 function [xlow,xupp,Flow,Fupp,A,iAfun,jAvar,iGfun,jGvar] = Setup_Problem()

% Defines the constraints for the problem:
%   maximize F(1)  (the objective row)
%   subject to
%            xlow <=   x  <= xupp
%            Flow <= F(x) <= Fupp

%Output for the command window
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('-----Problem Size-----')
fprintf('\nNumber of dynamic feasibility constraints: %d\n',nStates*nSteps);
fprintf('Number of control input constraints: %d \n',nSteps);
fprintf('Number of state constraints:%d \n',2*nSteps);
fprintf('Number of boundary conditions:%d \n\n',0);

%Number of constraint functions plus one for objective. Sum of above+1
size_func=nStates*nSteps+nSteps+2*nSteps;

%Number of decision variables
decision_vars=length(y0);


%Check size of problem
if decision_vars > 300 || size_func >300
    fprintf('\n \nToo many decision variables or constraints for student version of snopt\n \n')
    fprintf('Decision variables: %d \n',decision_vars)
    fprintf('Constraints: %d \n',size_func)
    fprintf('.....Press ctrl+c..... \n')
    pause
end


% The default objective row
Obj    =  1; 

%Init. Flow and Fupp
Flow = zeros(size_func,1);    Fupp = ones (size_func,1);


% Dynamic feasibility leads to equality constraints
Flow(1:nStates*nSteps)=0;
Fupp(1:nStates*nSteps)=0;

% Control constraints
Flow(nStates*nSteps+1:nStates*nSteps+nSteps)=-Inf;
Fupp(nStates*nSteps+1:nStates*nSteps+nSteps)=0;

% State constraints
Flow(nStates*nSteps+nSteps+1:nStates*nSteps+nSteps+2*nSteps)=-Inf;
Fupp(nStates*nSteps+nSteps+1:nStates*nSteps+nSteps+2*nSteps)=0;

% %BC equality constraints
% Flow(nStates*nSteps+nSteps+2*nSteps+1)=0;
% Fupp(nStates*nSteps+nSteps+2*nSteps+1)=0;

%Stack in the objective row (row 1)
Flow=[-Inf;Flow];
Fupp=[Inf;Fupp];

% No state constraints. They are included in Fupp and Flow
xlow = -Inf*ones(decision_vars,1);  xupp =  Inf*ones(decision_vars,1);


% Constant Jacobian elements are not treated specially.
iAfun = [];  jAvar = [];  A     = [];

% Coordinates of func jacobian that are nonzero: assuming none
[iGfun,jGvar] = find(ones(size_func,decision_vars));

 end
 end
 
