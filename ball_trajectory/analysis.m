clear all
close all
clc

%% Optimization
N = 30;  % time steps
num_links = 8;  % 1,2,4,8 (8 doesn't work, too many links for drake)
pend_length = 0.32;  % needs to match total length to ball in urdf
link_length = pend_length/num_links;
[utraj,xtraj,prog,r,goal_pos] = ball_trajectory(num_links,pend_length,N);

%% Analysis
% get state over time
time = xtraj.tspan(1):0.01:xtraj.tspan(2);
x_t = xtraj.eval(time);

% get ball trajectory
ball_t = zeros(3,length(x_t));

for i=1:length(x_t)
  q = x_t(1:r.getNumPositions,i);
  kinsol = r.doKinematics(q);
  [ball_pos,dBall_pos] = r.forwardKin(kinsol,findFrameId(r,'ball_com'),[0;0;-link_length]);
  ball_t(1:3,i) = ball_pos;
end

%% PLOT
% quad x,y,z
% figure
% hold on
% 
% plot(time,x_t(1,:),'r-')
% plot(time,x_t(2,:),'g-')
% plot(time,x_t(3,:),'b-')
% title('Quad Position Vs. Time')
% xlabel('Time [s]')
% ylabel('Position [m]')
% legend('x','y','z')
% hold off

% ball x,y,z
% figure
% hold on
% 
% inds = N/2:N;
% plot(time,ball_t(1,:),'r-')
% plot(time(end/2:end),goal_pos(1,inds),'r*')
% 
% plot(time,ball_t(2,:),'g-')
% plot(time(end/2:end),goal_pos(2,inds),'g*')
% 
% plot(time,ball_t(3,:),'b-')
% plot(time(end/2:end),goal_pos(3,inds),'b*')
% 
% title('Ball Position Vs. Time')
% xlabel('Time [s]')
% ylabel('Position [m]')
% legend('x','x-ref','y','y-ref','z','z-ref')
% 
% hold off

% 3d plot of quad and ball
figure
hold on

plot3(x_t(1,:),x_t(2,:),x_t(3,:),'b--')
plot3(ball_t(1,:),ball_t(2,:),ball_t(3,:),'r-')
plot3(goal_pos(1,N/2:N),goal_pos(2,N/2:N),goal_pos(3,N/2:N),'b*')
scatter3(goal_pos(1,1),goal_pos(2,1),goal_pos(3,1),100,'g','filled')
scatter3(goal_pos(1,end),goal_pos(2,end),goal_pos(3,end),100,'r','filled')

title('Ball and Reference Position Vs. Time')
xlabel('X-Position [m]')
ylabel('Y-Position [m]')
zlabel('Z-Position [m]')
legend('quad','ball','ball-target','Location','NorthWest')

hold off