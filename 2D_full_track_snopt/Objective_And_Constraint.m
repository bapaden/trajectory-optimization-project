 function [F,G] = Objective_And_Constraint(x)
 
 
%% use setup specific variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
 
 
 
%% Evaluate f(x) and Df(x) with automatic differentiation   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

y_obj = myAD(x); %convert x from a double array to a type myAD array
eval_result=megaFunction(y_obj); % evaluate mega-function at x

F=getvalue(eval_result); % extract value of mega-function at x
G=getderivs(eval_result); % extract jacobian of mega-function at x


%% func of type myAD returned from megaFunction gets passed to snopt as F=getvalues(func), G=getderivs(func).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

     function func = megaFunction(decision_var)%[cost ceq c bc]'
         
        y=[IC';decision_var];%augment decision-variable with initial condition
        
        %dynamic feasibility constraint 
        for i=1:(nSteps)
             %0=-x1[n+1]+x1[n]+dt*f1(x1[n],x2[n],...xk[n],u1[n],u2[n],...,up[n])
             dyn_con(nStates*(i-1)+1)=-y(nStates*(i)+1)+y(nStates*(i-1)+1)+dt*f1(y(nStates*(i-1)+1),y(nStates*(i-1)+2),y(nStates*(i-1)+3),y(nStates*(i-1)+4),y(nStates*(nSteps+1)+nControls*(i-1)+1),y(nStates*(nSteps+1)+nControls*(i-1)+2));
             %0=-x2[n+1]+x2[n]+dt*f2(x1[n],x2[n],...xk[n],u1[n],u2[n],...,up[n])
             dyn_con(nStates*(i-1)+2)=-y(nStates*(i)+2)+y(nStates*(i-1)+2)+dt*f2(y(nStates*(i-1)+1),y(nStates*(i-1)+2),y(nStates*(i-1)+3),y(nStates*(i-1)+4),y(nStates*(nSteps+1)+nControls*(i-1)+1),y(nStates*(nSteps+1)+nControls*(i-1)+2));
             %0=-x3[n+1]+x3[n]+dt*f3(x1[n],x2[n],...xk[n],u1[n],u2[n],...,up[n])
             dyn_con(nStates*(i-1)+3)=-y(nStates*(i)+3)+y(nStates*(i-1)+3)+dt*f3(y(nStates*(i-1)+1),y(nStates*(i-1)+2),y(nStates*(i-1)+3),y(nStates*(i-1)+4),y(nStates*(nSteps+1)+nControls*(i-1)+1),y(nStates*(nSteps+1)+nControls*(i-1)+2));
             %0=-x3[n+1]+x3[n]+dt*f3(x1[n],x2[n],...xk[n],u1[n],u2[n],...,up[n])
             dyn_con(nStates*(i-1)+4)=-y(nStates*(i)+4)+y(nStates*(i-1)+4)+dt*f4(y(nStates*(i-1)+1),y(nStates*(i-1)+2),y(nStates*(i-1)+3),y(nStates*(i-1)+4),y(nStates*(nSteps+1)+nControls*(i-1)+1),y(nStates*(nSteps+1)+nControls*(i-1)+2));
        end
        
        %control limits
        for i=1:(nSteps)
         %0>=u1[n]^2+u2[n]^2-lim^2
         u_sat(i)=y(nStates*(nSteps+1)+nControls*(i-1)+1)^2+y(nStates*(nSteps+1)+nControls*(i-1)+2)^2-max_accel^2;
        end   
       
        %state constraints
        for i=1:nSteps
%          disp('i');disp(i);
%          disp('nStates*(i-1)+2)');disp(nStates*(i-1)+2);
%          disp('y(nStates*(i-1)+2)');disp(getvalue(y(nStates*(i-1)+2))) 
%          disp('y');disp(getvalue(y));
%          pause
         state_con(i)=y(nStates*(i)+2)-width/2;
         state_con(nSteps+i)=-y(nStates*(i)+2)-width/2;
        end
        state_con;
        %Performance objective
        cost = 0;
%         for i=1:nSteps+1
%             %maximize the integral of lateral velocity -> max. position
%             %cost=cost-1*y(nStates*i-1);%+(y(nStates*i-2)-6)^2;
%             
%         end
        cost=-y(nStates*nSteps+1);
%         bc=y(nStates*(nSteps)+2);

func=[cost;dyn_con;u_sat;state_con];
        

     end

 end
