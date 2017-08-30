function [new_node,control] = ExtendTree(nearest_node,sample)

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


d0=inf;
index=1;
theta=linspace(0,2*pi,control_res);
candidates=zeros(4);

for i=1:control_res



candidates(i,:) = nearest_node' + dt*[f1(nearest_node(1),nearest_node(2),nearest_node(3),nearest_node(4),max_accel*cos(theta(i)),max_accel*sin(theta(i)));
f2(nearest_node(1),nearest_node(2),nearest_node(3),nearest_node(4),max_accel*cos(theta(i)),max_accel*sin(theta(i)));
f3(nearest_node(1),nearest_node(2),nearest_node(3),nearest_node(4),max_accel*cos(theta(i)),max_accel*sin(theta(i)));
f4(nearest_node(1),nearest_node(2),nearest_node(3),nearest_node(4),max_accel*cos(theta(i)),max_accel*sin(theta(i)))] ;%do an euler step


if norm(candidates(i,:)-sample(:)')<d0 %if the distance is less than the min distance, update the min distance
index=i; %note the index of the new node
d0=norm(candidates(i,:)-sample(:)'); %note the new min distance
end

end

% %try no control
% candidates(control_res+1,:) = nearest_node' + dt*[f1(nearest_node(1),nearest_node(2),nearest_node(3),nearest_node(4),0,0);
% f2(nearest_node(1),nearest_node(2),nearest_node(3),nearest_node(4),0,0);
% f3(nearest_node(1),nearest_node(2),nearest_node(3),nearest_node(4),0,0);
% f4(nearest_node(1),nearest_node(2),nearest_node(3),nearest_node(4),0,0)] ;

%pick best max control
control=max_accel*[cos(theta(index)),sin(theta(index))];
new_node=candidates(index,:);

% %if better overwrite with no control
% if norm(candidates(control_res+1,:)-sample(:)')<d0 %if the distance is less than the min distance, update the min distance
% index=i; %note the index of the new node
% d0=norm(candidates(control_res+1,:)-sample(:)'); %note the new min distance
% end



%     function x1dot=f1(x1,x2,x3,x4,u1,u2)
%         x1dot=x3;
%     end
%     function x2dot=f2(x1,x2,x3,x4,u1,u2)
%         x2dot=x4;
%     end
%     function x3dot=f3(x,y,xd,yd,u1,u2)
% 
%     Ftan=u1;
%     Fperp=u2;
%     
%     x3dot=(-1).*xd.*(1+(-1).*y.*k(x)).^(-1).*((-1).*xd.*y.*dk(x)+(-1).*yd.* ...
%     k(x))+(1+(-1).*y.*k(x)).^(-1).*(Ftan.*m.^(-1)+xd.*yd.*k(x));
% 
%     end
%     function x4dot=f4(x,y,xd,yd,u1,u2)
% 
%     Ftan=u1;
%     Fperp=u2;
%     
%     x4dot=Fperp.*m.^(-1)+(-1).*xd.^2.*k(x).*(1+(-1).*y.*k(x));
% 
%     end

end