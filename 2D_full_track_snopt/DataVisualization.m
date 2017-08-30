function DataVisualization
close all
%%load quad flight data and generate animation of attitude and position measurements
clear;clc;
X_data=[];
Y_data=[];
Yaw_data=[];
%load X_data
%load Z_data
%load Roll_data

load state_feedback.mat
XX=state_feedback(:,1)
YY=state_feedback(:,2);
RR=state_feedback(:,3);
DD=state_feedback(:,7);
T=state_feedback(:,10);



%T=X_data(:,1);
%XX=X_data(:,2);
%ZZ=Z_data(:,2);
%RR=Roll_data(:,2);
n=length(T);

tt=linspace(0,T(end),n);
X=@(t)interp1(T,XX,t)
Y=@(t)interp1(T,YY,t)
Yaw=@(t)interp1(T,RR,t)




% figure(2);
% hold on
% plot(tt,qr_w,'.')
% plot(tt,qr_x,'.')
% plot(tt,qr_y,'.')
% plot(tt,qr_z,'.')
% NewData = flightTest292713;
% 
% %reasign hover to vectors. data is in lab frame coordinates
% N=length(NewData(:,1));
% S=50;
% time = NewData.timeus(S:N)/1000000;
% dt=time(2)-time(1);
% x_lab = NewData.x(S:N)-NewData.x(27);
% y_lab = NewData.y(S:N)-NewData.y(27);
% z_lab = NewData.z(S:N)-NewData.z(27);
% x_ref = NewData.x_ref(S:N);
% y_ref = NewData.y_ref(S:N);
% z_ref =NewData.z_ref(S:N)-NewData.z_ref(1);
% q_x = NewData.q_x(S:N);
% q_y = NewData.q_y(S:N);
% q_z = NewData.q_z(S:N);
% q_w = NewData.q_w(S:N);


% thrust_cmd = NewData.ci_t(S:N);
% pitch_cmd = NewData.ci_p(S:N);
% roll_cmd = NewData.ci_r(S:N);
% yaw_cmd = NewData.ci_y(S:N);

% q_ref = zeros(4,length(time));
% for i=1:length(time)
% [qr_w,qr_x,qr_y,qr_z]=euler2quat(yaw_cmd(i),pitch_cmd(i),roll_cmd(i),'xyz');

% end


%car square
a=3;
b=a/2
mark1 = [a,b];
mark2 = [-a,b];
mark3 = [-a,-b];
mark4 = [a,-b];





for i=1:length(tt)
mark1_traj(:,i)=R(RR(i))*mark1'+[XX(i); YY(i)];
mark2_traj(:,i)=R(RR(i))*mark2'+[XX(i); YY(i)];
mark3_traj(:,i)=R(RR(i))*mark3'+[XX(i); YY(i)];
mark4_traj(:,i)=R(RR(i))*mark4'+[XX(i); YY(i)];
%mark1_ref(:,i)=quatrotate(q_ref(i,:),mark1)+[x_ref(i) y_ref(i) z_ref(i)];
%mark2_ref(:,i)=quatrotate(q_ref(i,:),mark2)+[x_ref(i) y_ref(i) z_ref(i)];
%mark3_ref(:,i)=quatrotate(q_ref(i,:),mark3)+[x_ref(i) y_ref(i) z_ref(i)];
end

%initialize plot outside of loop
init=[mark1_traj(:,1),mark2_traj(:,1),mark3_traj(:,1),mark4_traj(:,1),mark1_traj(:,1)];


figure(3)
plot(init(1,:),init(2,:),'-o','MarkerSize', 3,'MarkerFaceColor','g', 'LineWidth', 1)
hold on
%plot(linspace(-100,100,3),-tan(10*pi/180)*linspace(-100,100,3)-2/cos(pi*10/180),'k','LineWidth',2)
xlabel('Meters')
ylabel('Meters')
%fh=plot([mark1_fb(1,1);mark2_fb(1,1);mark3_fb(1,1);mark1_fb(1,1)],[mark1_fb(2,1);mark2_fb(2,1);mark3_fb(2,1);mark1_fb(2,1)],[mark1_fb(3,1);mark2_fb(3,1);mark3_fb(3,1);mark1_fb(3,1)],'MarkerSize', 20, 'LineWidth', 1);
axis([-50 50 -50 50]);









% Get figure size
pos = get(gcf, 'Position');
width = pos(3); height = pos(4);



% fast forward speed for playback
ff=30;
% Preallocate data (for storing frame data)
mov = zeros(height, width, 1, length(tt), 'uint8');

for frameNumber = 1:length(tt)
    frame_i=[mark1_traj(:,frameNumber),mark2_traj(:,frameNumber),mark3_traj(:,frameNumber),mark4_traj(:,frameNumber),mark1_traj(:,frameNumber)];
    hold off
    plot(frame_i(1,:),frame_i(2,:),'-o','MarkerSize', 3,'MarkerFaceColor','g', 'LineWidth', 1)
    axis([-100 100 -100 100]);
    hold on
    if frameNumber<=length(tt)-20 && frameNumber>=20
    plot(XX(frameNumber-19:frameNumber+19),YY(frameNumber-19:frameNumber+19),'r')
    end
    xlabel('Meters')
    ylabel('Meters')
    
    title(sprintf('Time: %0.2f sec', tt(frameNumber)));
    f = getframe(gcf);

    % Create a colormap for the first frame. For the rest of the frames,
    % use the same colormap
    if frameNumber == 1
        [mov(:,:,1,frameNumber), map] = rgb2ind(f.cdata, 256, 'nodither');
    else
        mov(:,:,1,frameNumber) = rgb2ind(f.cdata, map, 'nodither');
    end
end

% Create animated GIF
imwrite(mov, map, 'animation.gif', 'DelayTime',(T(n)/n)/ff, 'LoopCount', inf);


function rotation = R(psi)
rotation=[cos(psi) -sin(psi);sin(psi) cos(psi)];
end
end