function RRT_init()

clear;clc;close all;
Problem_Parameters()

global global_data
load('Data_Bucket/params')

Domain_x = [-.25*track_length,track_length];
Domain_y = [-width/2,width/2];

% Start and goal positions
figure(1); clf; hold on
state_0=IC';
plot(state_0(1),state_0(2),'bo','MarkerFaceColor','b','MarkerSize',10);
xy_goal = [track_length;0;0;0]; plot(xy_goal(1),xy_goal(2),'go','MarkerFaceColor','g','MarkerSize',10); drawnow;
hold off

Node = cell(5,1);%{States,Parents,Node Depth,Controls,Flow Map on node}
Node{1}(1,:) = state_0;
Node{2}(1) = 0;
Node{3}(1) = 1;
Node{4}(1,:) = [0 0];
Node{5}(1,:) = state_0 +dt*[f1(state_0(1),state_0(2),state_0(3),state_0(4),0,0);f2(state_0(1),state_0(2),state_0(3),state_0(4),0,0);f3(state_0(1),state_0(2),state_0(3),state_0(4),0,0);f4(state_0(1),state_0(2),state_0(3),state_0(4),0,0)]
i = 1;






figure(1); hold on;
axis([Domain_x,Domain_y]);
 hxy = plot(0,0,'ro');
% RRT algorithm
j=2;
complete=0;
best_so_far=0;
while j<1000 && complete==0
 
      
x_rnd = rand(1)*track_length;
y_rnd = rand(1)*width-width/2;
xd_rnd = rand(1)*50;
yd_rnd = rand(1)*18-9;
sample=[x_rnd;y_rnd;xd_rnd;yd_rnd];
% if rand(1)<.05;
%     sample=[120 4 25 0]';
% end

    
%%% Find Nearest Neighbor
        closest_node_id = closestVertex(Node,sample);
        closest_node=Node{1}(closest_node_id,:);

%%% Add New Node
        [new_node,u] = ExtendTree(closest_node,sample);
        
        %check if goal has been reached
        if new_node(1)>track_length
            complete=1
        end
        
        %monitor progress
        if new_node(1)>best_so_far
            best_so_far=new_node(1)
            j
        end
        
        %reject new node of fails collision check
        if abs(new_node(2))>width/2
            continue;
        end
        
        flow_map = new_node+dt*[f1(new_node(1),new_node(2),new_node(3),new_node(4),0,0);f2(new_node(1),new_node(2),new_node(3),new_node(4),0,0);f3(new_node(1),new_node(2),new_node(3),new_node(4),0,0);f4(new_node(1),new_node(2),new_node(3),new_node(4),0,0)]';

        Node{1} = [Node{1}; new_node];%add node to tree
        Node{2} = [Node{2} closest_node_id];%save parent node id
        Node{3} = [Node{3} Node{3}(closest_node_id)+1];%save depth of node
        Node{4} = [Node{4};u];%save control
        Node{5} = [Node{5};flow_map];%save flow map on node
        
    delete(hxy);
    figure(1);
    hxy = plot(sample(1),sample(2),'r.');axis([Domain_x,Domain_y])
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
     
    % Plot extension (Comment the next few lines out if you want your code to
    % run a bit quicker. The plotting is useful for debugging though.)
    figure(1)
    hold on
    plot(new_node(1),new_node(2),'bo','MarkerFaceColor','b','MarkerSize',5);
    line([new_node(1),closest_node(1)],[new_node(2),closest_node(2)]);
    axis([Domain_x,Domain_y])

%Node{3}
j=j+1;
end

save('Data_Bucket/RRT_Tree','Node');
Process_RRT();
end
