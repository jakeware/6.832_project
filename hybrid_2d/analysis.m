clear all
close all
clc

%% Optimization
goal_pos = [1;2];
hybrid_ball_goal(goal_pos);

%% Analysis
% % get state over time
% time = xtraj.tspan(1):0.01:xtraj.tspan(2);
% x_t = xtraj.eval(time);
% 
% % get ball trajectory
% ball_t = zeros(3,length(x_t));
% 
% for i=1:length(x_t)
%   q = x_t(1:r.getNumPositions,i);
%   kinsol = r.doKinematics(q);
%   [ball_pos,dBall_pos] = r.forwardKin(kinsol,findFrameId(r,'ball_com'),[0;0;-link_length]);
%   ball_t(1:3,i) = ball_pos;
% end

%% PLOT
% % 3d plot of quad and ball
% figure
% hold on
% 
% plot3(x_t(1,:),x_t(2,:),x_t(3,:),'b--')
% plot3(ball_t(1,:),ball_t(2,:),ball_t(3,:),'r-')
% 
% plot3(x_t(1,1),x_t(2,1),x_t(3,1),'go','MarkerSize',12,'MarkerFaceColor','g')
% plot3(x_t(1,end),x_t(2,end),x_t(3,end),'r.','MarkerSize',20)
% 
% plot3(top_goal_pos(1,1),top_goal_pos(2,1),top_goal_pos(3,1),'k*','MarkerSize',12)
% plot3(fwd_goal_pos(1,end),fwd_goal_pos(2,end),fwd_goal_pos(3,end),'m*','MarkerSize',12)
% 
% title('Ball and Reference Position Vs. Time')
% xlabel('X-Position [m]')
% ylabel('Y-Position [m]')
% zlabel('Z-Position [m]')
% legend({'quad','ball','start','stop','ball-target1','ball-target2'},'Location','SouthWest')
% 
% axis([-3,3,-3,3])
% view(0,90)
% 
% hold off