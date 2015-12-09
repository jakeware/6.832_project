function [utraj,xtraj,prog,r] = ball_goal_forward

%% NOTES
%r.getStateFrame.getCoordinateNames  % print state variable names

%% Setup
plot_results = 0;

%% FUNCTION
num_links = 8;  % 1,2,4,8 (8 doesn't work, too many links for drake)
pend_length = 0.32;  % needs to match total length to ball in urdf
link_length = pend_length/num_links;
r = QuadrotorML(num_links);

N = 10;  % knot points
minimum_duration = .1;
maximum_duration = 5;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);  

% add constraint: initial state
x0 = Point(getStateFrame(r));  
x0.base_z = 0.5;
u0 = double(nominalThrust(r));

% quad bounding box
quad_bound_max = [2,1,2];
quad_bound_min = [-1,-1,pend_length];

% add constraint: Ball Goal state
goal_pos = [2.315;0;1];  % ball outside of box in x direction

% plan visualization
v = constructVisualizer(r);
v.draw(0,double(x0));
prog = addPlanVisualizer(r,prog);

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);

% add constraint: quad x-position
state_select = 1;
A = eye(N);
alt_lb = repmat(quad_bound_min(1),N,1);
alt_ub = repmat(quad_bound_max(1),N,1);
prog = prog.addStateConstraint(LinearConstraint(alt_lb,alt_ub,A),{1:N},state_select);

% add constraint: quad y-position
state_select = 2;
A = eye(N);
alt_lb = repmat(quad_bound_min(2),N,1);
alt_ub = repmat(quad_bound_max(2),N,1);
prog = prog.addStateConstraint(LinearConstraint(alt_lb,alt_ub,A),{1:N},state_select);

% add constraint: quad z-position
state_select = 3;
A = eye(N);
alt_lb = repmat(quad_bound_min(3),N,1);
alt_ub = repmat(quad_bound_max(3),N,1);
prog = prog.addStateConstraint(LinearConstraint(alt_lb,alt_ub,A),{1:N},state_select);

% final roll
state_select = 4;
A = 1;
rollf_lb = deg2rad(-10);
rollf_ub = deg2rad(10);
prog = prog.addStateConstraint(LinearConstraint(rollf_lb,rollf_ub,A),{N},state_select);

% final pitch
state_select = 5;
A = 1;
pitchf_lb = deg2rad(-10);
pitchf_ub = deg2rad(10);
prog = prog.addStateConstraint(LinearConstraint(pitchf_lb,pitchf_ub,A),{N},state_select);

% % roll and pitch constraint
% % running roll
% state_select = 4;
% A = eye(N-1);
% roll_lb = repmat(deg2rad(-60),N-1,1);
% roll_ub = repmat(deg2rad(60),N-1,1);
% prog = prog.addStateConstraint(LinearConstraint(roll_lb,roll_ub,A),{1:N-1},state_select);
% 
% % running pitch
% state_select = 5;
% A = eye(N-1);
% pitch_lb = repmat(deg2rad(-60),N-1,1);
% pitch_ub = repmat(deg2rad(60),N-1,1);
% prog = prog.addStateConstraint(LinearConstraint(pitch_lb,pitch_ub,A),{1:N-1},state_select);

% yaw constraint
% running yaw
state_select = 6;
A = eye(N);
yaw_lb = repmat(deg2rad(-15),N,1);
yaw_ub = repmat(deg2rad(15),N,1);
prog = prog.addStateConstraint(LinearConstraint(yaw_lb,yaw_ub,A),{1:N},state_select);

% % add constraint: theta 1
% state_select = 7;
% A = eye(N);
% theta1_lb = repmat(-pi/2,N,1);
% theta1_ub = repmat(pi/2,N,1);
% prog = prog.addStateConstraint(LinearConstraint(theta1_lb,theta1_ub,A),{1:N},state_select);
% 
% % add constraint: theta 2
% state_select = 8;
% A = eye(N);
% theta2_lb = repmat(-pi/2,N,1);
% theta2_ub = repmat(pi/2,N,1);
% prog = prog.addStateConstraint(LinearConstraint(theta2_lb,theta2_ub,A),{1:N},state_select);

% add constraint: goal position
goalConstraint = FunctionHandleConstraint([0;0;0],[0;0;0],r.getNumStates,@(x) final_state_con(r,x,goal_pos,link_length),1);
prog = prog.addStateConstraint(goalConstraint,{N});

% add costs
% prog = prog.addRunningCost(@cost);
% prog = prog.addFinalCost(@finalCost);

% initial trajectory
tf0 = 2;  % initial guess at time
xf = x0;                       
xf.base_x = 3;
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(u0);

% solve for trajectory
tic
[xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
[str,category] = snoptInfo(info)
toc

% create visualization
v.playback(xtraj,struct('slider',true));

%% ANALYSIS
% get state over time
time = xtraj.tspan(1):0.01:xtraj.tspan(2);
x_t = xtraj.eval(time);

% get ball trajectory
ball_t = zeros(3,length(x_t));

for i=1:length(x_t)
  q = x_t(1:7,i);
  kinsol = r.doKinematics(q);
  [ball_pos,dBall_pos] = r.forwardKin(kinsol,findFrameId(r,'ball_com'),[0;0;-0.3]);
  ball_t(1:3,i) = ball_pos;
end

%% PLOT
if plot_results
  % quad x,y,z
  figure
  hold on

  plot(time,x_t(1,:),'r-')
  plot(time,x_t(2,:),'g-')
  plot(time,x_t(3,:),'b-')
  title('Quad Position Over Time')
  xlabel('Time [s]')
  ylabel('Position [m]')
  legend('x','y','z')
  hold off

  % ball x,y,z
  figure
  hold on

  plot(time,ball_t(1,:),'r-')
  plot(time,ball_t(2,:),'g-')
  plot(time,ball_t(3,:),'b-')
  title('Ball Position Over Time')
  xlabel('Time [s]')
  ylabel('Position [m]')
  legend('x','y','z')

  hold off

  % 3d plot of quad and ball
  figure
  hold on

  plot3(x_t(1,:),x_t(2,:),x_t(3,:),'b-')
  plot3(ball_t(1,:),ball_t(2,:),ball_t(3,:),'r-')

  title('Quad and Ball Position Over Time')
  xlabel('X-Position [m]')
  ylabel('Y-Position [m]')
  zlabel('Z-Position [m]')
  legend('quad','ball')

  hold off
end
end

function [f,df] = final_state_con(obj,x,goal_pos,link_length)
  q = x(1:obj.getNumStates/2);
  kinsol = obj.doKinematics(q);
  [ball_pos,dBall_pos] = obj.forwardKin(kinsol,findFrameId(obj,'ball_com'),[0;0;-link_length]);

  f = ball_pos - goal_pos;
  df = [dBall_pos zeros(3,obj.getNumStates/2)];
end

% cost function
function [g,dg] = cost(dt,x,u)
  R = eye(4);
  g = u'*R*u;
  dg = [zeros(1,1+size(x,1)),2*u'*R];
end

% final cost function
function [h,dh] = finalCost(t,x)
  h = t;
  dh = [1,zeros(1,size(x,1))];
end

