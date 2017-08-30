function Process_RRT()

%process rrt result
clear;clc;
load('Data_Bucket\RRT_Tree')
load('Data_Bucket\params')
traj=Node{1};
parent=Node{2};
control=Node{4};
performance=traj(:,1);
[best_node index]=max(performance)

i=1;
while index>0
    x_rev(i,:)=traj(index,:);
    u_rev(i,:)=control(index,:);
    index=parent(index);
    i=i+1;
end
X=flipud(x_rev)
U=flipud(u_rev);
U=U(2:end,:)

save('Data_Bucket/init_traj_rrt','X')
save('Data_Bucket/init_control_rrt','U')
Analyze_Results('initial_guess_rrt')

end

