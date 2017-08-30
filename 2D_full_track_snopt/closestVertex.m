function closest_node_id = closestVertex(Node,sample)

%use global variables
global global_data
% nStates=global_data(1);
nSteps=global_data(2);
% nControls=global_data(3);
% dt=global_data(4);
% max_accel=global_data(5);
% mass=global_data(6);
% grav=global_data(7);
% mu_fric=global_data(8);
% width=global_data(9);
% control_res=global_data(10);
% initial_guess=global_data(11);
% IC=global_data(12:end);

d_min=inf;
N=length(Node{1}(:,1));
% for i=1:N
%     if Node{3}(i)*dt>time
%         continue;
%     end
%     compare_node = Node{1}(i,:);%pull node id i from cell
%     if norm(compare_node'-sample)<=d_min
%         d_min=norm(compare_node'-sample);
%         closest_node_id = i;
%     end
% end

for i=1:N
    %don't compare with nodes whose depth is greater than the limit
    if Node{3}(i)>(nSteps)
        continue;
    end
    compare_node = Node{5}(i,:);%pull node id i from cell
    if norm(compare_node'-sample)<=d_min
        d_min=norm(compare_node'-sample);
        closest_node_id = i;
    end
end

end