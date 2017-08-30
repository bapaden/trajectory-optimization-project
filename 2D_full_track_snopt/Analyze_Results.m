function Analyze_Results(dataset)

close all
if strcmp(dataset,'initial_guess_determinsitic')

    load('Data_Bucket\init_traj_deterministic')
    load('Data_Bucket\init_control_deterministic')
    load('Data_Bucket\params')
   % time=tspan;
end

if strcmp(dataset,'initial_guess_rrt')
    
    load('Data_Bucket\init_traj_rrt')
    load('Data_Bucket\init_control_rrt')
    load('Data_Bucket\params')
    
end

if strcmp(dataset,'optimization_result') || isempty(dataset);
    
    load('Data_Bucket\optimization_traj')
    load('Data_Bucket\optimization_control')
    load('Data_Bucket\params')
    %time=tspan;
    
end





%put sim data into local variables
x=mod(X(:,1),3*track_length);
y=X(:,2);
xd=X(:,3);
yd=X(:,4);




%compute track from track data
x0=[0;0;0];
f=@(t,y) [k(t);
    cos(y(1));
    sin(y(1))];
track_length
xspan=linspace(0,track_length,1000);
Y=ode5(f,xspan,x0);

%car trajectory in track coordinate frame
x_interp=@(t) interp1(tspan,x,t);%interpolation of track frame trajectory
y_interp=@(t) interp1(tspan,y,t);%interpolation of track frame trajectory
xt_interp=@(x) interp1(xspan,Y(:,2),x);%interpolation of track x-coordinate in euclidean space
yt_interp=@(x) interp1(xspan,Y(:,3),x);%interpolation of track y-coordinate in euclidean space

%control forces. No control defined on last step
u1_interp=@(t) interp1(tspan(1:nSteps),U(:,1),t);
u2_interp=@(t) interp1(tspan(1:nSteps),U(:,2),t);

%Angle of track coordinate frame relative to euclidean coordinate frame
theta_interp=@(x) interp1(xspan,Y(:,1),x);

%car trajectory in euclidean coordinate frame

xtraj=xt_interp(x_interp(tspan))-sin(theta_interp(x_interp(tspan))).*y_interp(tspan);
ytraj=yt_interp(x_interp(tspan))+cos(theta_interp(x_interp(tspan))).*y_interp(tspan);
left_bound_x=xt_interp(xspan)-sin(theta_interp(xspan)).*width/2;
left_bound_y=yt_interp(xspan)+cos(theta_interp(xspan)).*width/2;
right_bound_x=xt_interp(xspan)+sin(theta_interp(xspan)).*width/2;
right_bound_y=yt_interp(xspan)-cos(theta_interp(xspan)).*width/2;
u1=u1_interp(tspan);
u2=u2_interp(tspan);



figure(1)
subplot(2,1,1)
plot(tspan,x_interp(tspan))
xlabel('time')
ylabel('down track position')
subplot(2,1,2)
plot(tspan,y_interp(tspan))
xlabel('time')
ylabel('cross track position')

figure(4)
subplot(2,1,1)
plot(tspan,xd)
ylabel('downtrack velocity')
xlabel('ttime')
subplot(2,1,2)
plot(tspan,yd)
ylabel('cross track velocity')
xlabel('time')
figure(3)
subplot(2,1,1)
plot(tspan,u1)
xlabel('time')
ylabel('longitudinal acceleration')
subplot(2,1,2)
plot(tspan,u2)
xlabel('time')
ylabel('lateral acceleration')




trackFig=figure(5);
set(trackFig, 'Position', [200 100 1000 700])

subplot(2,1,1)
plot(xt_interp(xspan),yt_interp(xspan))
axis([0 455  -75 175])
title('Car trajectory: World coordinates','Interpreter','Latex','FontSize',18)
xlabel('Meters','Interpreter','Latex','FontSize',18)
ylabel('Meters','Interpreter','Latex','FontSize',18)
hold on
plot(xtraj,ytraj,'g*-')
plot(left_bound_x,left_bound_y,'r')
plot(right_bound_x,right_bound_y,'r')
subplot(2,1,2)
hold on
plot(x_interp(tspan),y_interp(tspan),'g*-')
plot([x_interp(tspan(1)),x_interp(tspan(end))],[width/2,width/2],'r')
plot([x_interp(tspan(1)),x_interp(tspan(end))],[-width/2,-width/2],'r')
plot([x_interp(tspan(1)),x_interp(tspan(end))],[0,0],'b')
title('Car trajectory: Track coordinates','Interpreter','Latex','FontSize',18)
xlabel('Downtrack Position (Meters)','Interpreter','Latex','FontSize',18)
ylabel('Crosstrack Position (Meters)','Interpreter','Latex','FontSize',18)

%save track data for animation



end

