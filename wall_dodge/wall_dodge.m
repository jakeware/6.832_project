function [utraj,xtraj,prog,r,obs_con_pos] = wall_dodge(num_links,pend_length,N,wall_node)

%% NOTES
%r.getStateFrame.getCoordinateNames  % print state variable names

%% Setup
max_z = 1.25;  % max quad height
link_length = pend_length/num_links;

% initial and final state
start_pos = [0;0;0.5];
goal_pos = [6;0;0.5];

% time setup

minimum_duration = .1;
maximum_duration = 10;

%% FUNCTION
% setup
r = QuadrotorML(num_links);
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);  

% add obstacle
obs_size = [0.2,2,1.0];
obs_pos = [goal_pos(1)/2,0,obs_size(3)/2];
rpy = zeros(3,1);
r = addObstacle(r,obs_size,obs_pos,rpy);
obs_con_pos = obs_pos';
obs_con_pos(3) = 1.05;

% add constraint: initial state
x0 = Point(getStateFrame(r));  
x0.base_z = start_pos(3);
u0 = double(nominalThrust(r));
prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);

% initial trajectory
tf0 = 10;  % initial guess at time

% intermediate pose
xm = x0;
xm.base_x = goal_pos(1)/2;
xm.base_y = goal_pos(2);
xm.base_z = max_z;

% final pose
xf = x0;                       
xf.base_x = goal_pos(1);
xf.base_y = goal_pos(2);
xf.base_z = goal_pos(3);

pp1 = PPTrajectory(foh([0,tf0/2],[double(x0),double(xm)]));
pp2 = PPTrajectory(foh([tf0/2,tf0],[double(xm),double(xf)]));
traj_init.x = pp1.append(pp2);
traj_init.u = ConstantTrajectory(u0);

% draw initial trajectory
v = constructVisualizer(r);
v.draw(0,double(x0));
prog = addPlanVisualizer(r,prog);

% add constraint: final roll
state_select = 4;
A = 1;
rollf_lb = deg2rad(-5);
rollf_ub = deg2rad(5);
prog = prog.addStateConstraint(LinearConstraint(rollf_lb,rollf_ub,A),{N},state_select);

% add constraint: final pitch
state_select = 5;
A = 1;
pitchf_lb = deg2rad(-5);
pitchf_ub = deg2rad(5);
prog = prog.addStateConstraint(LinearConstraint(pitchf_lb,pitchf_ub,A),{N},state_select);

% add constraint: running roll
state_select = 4;
A = eye(N-1);
roll_lb = repmat(deg2rad(-5),N-1,1);
roll_ub = repmat(deg2rad(5),N-1,1);
prog = prog.addStateConstraint(LinearConstraint(roll_lb,roll_ub,A),{1:N-1},state_select);

% add constraint: running pitch
state_select = 5;
A = eye(N-1);
pitch_lb = repmat(deg2rad(-45),N-1,1);
pitch_ub = repmat(deg2rad(45),N-1,1);
prog = prog.addStateConstraint(LinearConstraint(pitch_lb,pitch_ub,A),{1:N-1},state_select);

% add constraint: running yaw
state_select = 6;
A = eye(N-1);
yaw_lb = repmat(deg2rad(-5),N-1,1);
yaw_ub = repmat(deg2rad(5),N-1,1);
prog = prog.addStateConstraint(LinearConstraint(yaw_lb,yaw_ub,A),{1:N-1},state_select);

% add constraint: z-position
state_select = 3;
A = eye(N);
alt_lb = repmat(pend_length,N,1);
alt_ub = repmat(max_z,N,1);
prog = prog.addStateConstraint(LinearConstraint(alt_lb,alt_ub,A),{1:N},state_select);

% add constraint: y-position
state_select = 2;
A = eye(N);
y_lb = repmat(-0.1,N,1);
y_ub = repmat(0.1,N,1);
prog = prog.addStateConstraint(LinearConstraint(y_lb,y_ub,A),{1:N},state_select);

% add constraint: x-position
% nodes before wall
state_select = 1;
size_A = wall_node;
A = eye(size_A);
x_lb = repmat(start_pos(1) - 5,size_A,1);
x_ub = repmat(goal_pos(1)/2,size_A,1);
prog = prog.addStateConstraint(LinearConstraint(x_lb,x_ub,A),{1:wall_node},state_select);

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

% nodes after wall
state_select = 1;
size_A = N - wall_node;
A = eye(size_A);
x_lb = repmat(goal_pos(1)/2,size_A,1);
x_ub = repmat(goal_pos(1) + 5,size_A,1);
prog = prog.addStateConstraint(LinearConstraint(x_lb,x_ub,A),{wall_node+1:N},state_select);

% add constraint: enforce knot point over wall
state_select = 1;
A = 1;
knot_lb = obs_pos(1) - 0.01;
knot_ub = obs_pos(1) + 0.01;
prog = prog.addStateConstraint(LinearConstraint(knot_lb,knot_ub,A),{wall_node},state_select);

% add constraint: old obstacles
% collision_constraint = generateConstraint(MinDistanceConstraint(r,0.01),0);
% prog = prog.addStateConstraint(collision_constraint{1},1:N,1:getNumPositions(r));

% add constraint: new obstacle constraint on pendulum height
obs_lb = [-Inf;-Inf;obs_con_pos(3)];
obs_ub = [Inf;Inf;max_z];
obsConstraint = FunctionHandleConstraint(obs_lb,obs_ub,r.getNumStates(),@(x) obs_state_con(r,x,obs_con_pos,link_length),1);
prog = prog.addStateConstraint(obsConstraint,{wall_node});

% add constraint: goal position
goalConstraint = FunctionHandleConstraint([0;0;0],[0;0;0],r.getNumStates(),@(x) final_state_con(r,x,goal_pos,link_length),1);
prog = prog.addStateConstraint(goalConstraint,{N});

% add costs
% prog = prog.addRunningCost(@cost);
% prog = prog.addFinalCost(@finalCost);

% solve for trajectory
tic
[xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
[str,category] = snoptInfo(info)
toc

% create visualization
v.playback(xtraj,struct('slider',true));

end

function [f,df] = final_state_con(obj,x,goal_pos,link_length)
  q = x(1:obj.getNumStates/2);
  kinsol = obj.doKinematics(q);
  [ball_pos,dBall_pos] = obj.forwardKin(kinsol,findFrameId(obj,'ball_com'),[0;0;-link_length]);

  f = ball_pos - goal_pos;
  df = [dBall_pos zeros(3,obj.getNumStates/2)];
end

function [f,df] = obs_state_con(obj,x,obs_con_pos,link_length)
  q = x(1:obj.getNumStates/2);
  kinsol = obj.doKinematics(q);
  [ball_pos,dBall_pos] = obj.forwardKin(kinsol,findFrameId(obj,'ball_com'),[0;0;-link_length]);

  f = ball_pos - obs_con_pos;
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

