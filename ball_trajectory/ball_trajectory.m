function [utraj,xtraj,prog,r,goal_pos] = ball_trajectory(num_links,pend_length,N)

%% NOTES
%r.getStateFrame.getCoordinateNames  % print state variable names

%% FUNCTION
link_length = pend_length/num_links;
r = QuadrotorML(num_links);

minimum_duration = .1;
maximum_duration = 6;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);  

% add constraint: initial state
x0 = Point(getStateFrame(r));  
x0.base_z = 2;
u0 = double(nominalThrust(r));

% goal trajectory
goal_pos(1,:) = linspace(0,2*pi - 0.01,N);
goal_pos(2,:) = sin(1.5*goal_pos(1,:));
goal_pos(3,:) = sin(1.2*goal_pos(1,:)) + x0.base_z - pend_length;

% plan visualization
v = constructVisualizer(r);
v.draw(0,double(x0));
prog = addPlanVisualizer(r,prog);

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);

% add constraint: z-position
state_select = 3;
A = eye(N);
alt_lb = repmat(pend_length,N,1);
alt_ub = repmat(5,N,1);
prog = prog.addStateConstraint(LinearConstraint(alt_lb,alt_ub,A),{1:N},state_select);

% add constraint: y-position
state_select = 2;
A = eye(N);
y_lb = repmat(-5,N,1);
y_ub = repmat(5,N,1);
prog = prog.addStateConstraint(LinearConstraint(y_lb,y_ub,A),{1:N},state_select);

% add constraint: x-position
% nodes before wall
state_select = 1;
A = eye(N);
x_lb = repmat(-5,N,1);
x_ub = repmat(10,N,1);
prog = prog.addStateConstraint(LinearConstraint(x_lb,x_ub,A),{1:N},state_select);

% roll and pitch constraint
% running roll
state_select = 4;
A = eye(N);
roll_lb = repmat(deg2rad(-60),N,1);
roll_ub = repmat(deg2rad(60),N,1);
prog = prog.addStateConstraint(LinearConstraint(roll_lb,roll_ub,A),{1:N},state_select);

% running pitch
state_select = 5;
A = eye(N);
pitch_lb = repmat(deg2rad(-60),N,1);
pitch_ub = repmat(deg2rad(60),N,1);
prog = prog.addStateConstraint(LinearConstraint(pitch_lb,pitch_ub,A),{1:N},state_select);

% yaw constraint
% running yaw
% state_select = 6;
% A = eye(N);
% yaw_lb = repmat(deg2rad(-5),N,1);
% yaw_ub = repmat(deg2rad(5),N,1);
% prog = prog.addStateConstraint(LinearConstraint(yaw_lb,yaw_ub,A),{1:N},state_select);

% add constraint: theta 1
state_select = 7;
A = eye(N);
theta1_lb = repmat(-pi/2,N,1);
theta1_ub = repmat(pi/2,N,1);
prog = prog.addStateConstraint(LinearConstraint(theta1_lb,theta1_ub,A),{1:N},state_select);

% add constraint: theta 2
state_select = 8;
A = eye(N);
theta2_lb = repmat(-pi/2,N,1);
theta2_ub = repmat(pi/2,N,1);
prog = prog.addStateConstraint(LinearConstraint(theta2_lb,theta2_ub,A),{1:N},state_select);

% add constraint: goal position
for i=N/2:N
  gp_lb = [0;0;0];
  gp_ub = [0;0;0];
  goalConstraint = FunctionHandleConstraint(gp_lb,gp_ub,r.getNumStates,@(x) state_con(r,x,goal_pos(:,i),link_length),1);
  prog = prog.addStateConstraint(goalConstraint,{i});
end

% add costs
% prog = prog.addRunningCost(@cost);
% prog = prog.addFinalCost(@finalCost);

% initial trajectory
tf0 = 2;  % initial guess at time
xf = x0;                       
xf.base_x = 6;
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(u0);

% solve for trajectory
tic
[xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
[str,category] = snoptInfo(info)
toc

% create visualization
v.playback(xtraj,struct('slider',true));

end

function [f,df] = state_con(obj,x,goal_pos,link_length)
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

