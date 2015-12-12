clear all
close all
clc

%% Optimization
plot_results = 1;

N = 10;  % must be even
wall_node = 6;
num_links = 8;  % 1,2,4,8
pend_length = 0.32;  % needs to match total length to ball in urdf
link_length = pend_length/num_links;

[utraj,xtraj,prog,r,obs_con_pos] = wall_dodge(num_links,pend_length,N,wall_node);

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
if plot_results
  figure
  hold on

  plot3(x_t(1,:),x_t(2,:),x_t(3,:),'b--')
  plot3(ball_t(1,:),ball_t(2,:),ball_t(3,:),'r-')

  plot3(x_t(1,1),x_t(2,1),x_t(3,1),'go','MarkerSize',10,'MarkerFaceColor','g')
  plot3(x_t(1,end),x_t(2,end),x_t(3,end),'ro','MarkerSize',10,'MarkerFaceColor','r')

  plot3(obs_con_pos(1,1),obs_con_pos(2,1),obs_con_pos(3,1),'k*','MarkerSize',12)

  title('Quad and Ball Position Vs. Time')
  xlabel('X-Position [m]')
  ylabel('Y-Position [m]')
  zlabel('Z-Position [m]')
  legend({'quad','ball','start','stop','z_obs'},'Location','NorthWest')

  view(0,0)

  hold off
end