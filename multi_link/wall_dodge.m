function [utraj,xtraj,prog,r] = wall_dodge

%% NOTES
%r.getStateFrame.getCoordinateNames  % print state variable names

%% TODO

%% Questions
% How do we add a knot point at a specific xyz location?
% Why doesn't the vehicle avoid the obstacle without cost functions?

% 1) Ball to goal region
% 2) Ball along trajectory
% 3) Swing over obstacle

%% FUNCTION
% initial and final state
start_pos = [0;0;0.5];
goal_pos = [6;0;0.5];

% pendulum length
pend_len = 0.3;

% time setup
N = 30;  % must be even
minimum_duration = .1;
maximum_duration = 5;

% setup
r = Quadrotor();
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);  

% add obstacle
size = [0.3,2,1.0];
obs_pos = [goal_pos(1)/2,0,size(3)/2];
rpy = zeros(3,1);
r = addObstacle(r,size,obs_pos,rpy);

% add constraint: initial state
x0 = Point(getStateFrame(r));  
x0.base_z = start_pos(3);
u0 = double(nominalThrust(r));
prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);

% initial trajectory
tf0 = 10;  % initial guess at time
xf = x0;                       
xf.base_x = goal_pos(1);

xm = x0;
xm.base_x = goal_pos(1)/2;
xm.base_z = 1.25;

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

% add constraint: roll and pitch
% % roll
% state_select = 4;
% A = eye(N-1);
% roll_lb = repmat(deg2rad(-45),N-1,1);
% roll_ub = repmat(deg2rad(45),N-1,1);
% prog = prog.addStateConstraint(LinearConstraint(roll_lb,roll_ub,A),{1:N-1},state_select);
% 
% % pitch
% state_select = 5;
% A = eye(N-1);
% pitch_lb = repmat(deg2rad(-45),N-1,1);
% pitch_ub = repmat(deg2rad(45),N-1,1);
% prog = prog.addStateConstraint(LinearConstraint(pitch_lb,pitch_ub,A),{1:N-1},state_select);

% add constraint: z-position
state_select = 3;
A = eye(N);
alt_lb = repmat(pend_len,N,1);
alt_ub = repmat(1.25,N,1);
prog = prog.addStateConstraint(LinearConstraint(alt_lb,alt_ub,A),{1:N},state_select);

% add constraint: y-position
state_select = 2;
A = eye(N);
y_lb = repmat(-0.1,N,1);
y_ub = repmat(0.1,N,1);
prog = prog.addStateConstraint(LinearConstraint(y_lb,y_ub,A),{1:N},state_select);

% add constraint: x-position
state_select = 1;
A = eye(N/2);
y_lb = repmat(start_pos(1) - 5,N/2,1);
y_ub = repmat(goal_pos(1)/2,N/2,1);
prog = prog.addStateConstraint(LinearConstraint(y_lb,y_ub,A),{1:N/2},state_select);

state_select = 1;
A = eye(N/2);
x_lb = repmat(goal_pos(1)/2,N/2,1);
x_ub = repmat(goal_pos(1) + 5,N/2,1);
prog = prog.addStateConstraint(LinearConstraint(x_lb,x_ub,A),{N/2+1:N},state_select);

% add constraint: enforce knot point over wall
state_select = 1;
A = 1;
x_lb = obs_pos(1) - 0.01;
x_ub = obs_pos(1) + 0.01;
prog = prog.addStateConstraint(LinearConstraint(x_lb,x_ub,A),{N/2},state_select);

% add constraint: obstacles
collision_constraint = generateConstraint(MinDistanceConstraint(r,0.01),0);
prog = prog.addStateConstraint(collision_constraint{1},1:N,1:getNumPositions(r));

% add constraint: goal position
goalConstraint = FunctionHandleConstraint([0;0;0],[0;0;0],r.getNumStates(),@(x) final_state_con(r,x,goal_pos),1);
prog = prog.addStateConstraint(goalConstraint,{N});

% add costs
prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

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
  q = x_t(1:r.getNumStates()/2,i);
  kinsol = r.doKinematics(q);
  [ball_pos,dBall_pos] = r.forwardKin(kinsol,findFrameId(r,'ball_com'),[0;0;-0.3]);
  ball_t(1:3,i) = ball_pos;
end

%% PLOT
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

function [f,df] = final_state_con(obj,x,goal_pos)
  q = x(1:obj.getNumStates());
  kinsol = obj.doKinematics(q);
  [ball_pos,dBall_pos] = obj.forwardKin(kinsol,findFrameId(obj,'ball_com'),[0;0;-0.3]);

  f = ball_pos - goal_pos;
  df = [dBall_pos zeros(3,obj.getNumStates()/2)];
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

