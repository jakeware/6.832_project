function [utraj,xtraj,prog,r] = runDircol

%% NOTES
%r.getStateFrame.getCoordinateNames  % print state variable names

%% TODO
% Add constraints to enforce reasonable solutions
% Add second pivot to pendulum
% Limit thrust?

%% Questions
% How do we get the ball position over the solved trajectory?
% How do we define an obstacle constraint for the ball?
% What is a good hybrid system example to work from?

% 1) Ball to goal region
% 2) Ball along trajectory
% 3) Swing over obstacle

%% FUNCTION
% need to redefine this
r = Quadrotor();

% add obstacle
size = [0.3,2,1];
xyz = [3,0,size(3)/2];
rpy = zeros(3,1);
r = addObstacle(r,size,xyz,rpy);

pend_len = 0.3;  % pendulum length
N = 30;  % time steps
minimum_duration = .1;
maximum_duration = 20;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);  

% add constraint: initial state
x0 = Point(getStateFrame(r));  
x0.base_z = 0.5;
u0 = double(nominalThrust(r));

% plan visualization
v = constructVisualizer(r);
v.draw(0,double(x0));
prog = addPlanVisualizer(r,prog);

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);

% add constraint: final state
% xf = x0;                       
% xf.base_x = 6;
%prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N);
%prog = prog.addInputConstraint(ConstantConstraint(u0),N);

% add constraint: quad x,y,z goal region
% final x
% state_select = 1;
% A = 1;
% xf_lb = 5.5;
% xf_ub = 6.5;
% prog = prog.addStateConstraint(LinearConstraint(xf_lb,xf_ub,A),{N},state_select);
% 
% final y
% state_select = 2;
% A = 1;
% yf_lb = -0.5;
% yf_ub = 0.5;
% prog = prog.addStateConstraint(LinearConstraint(yf_lb,yf_ub,A),{N},state_select);
% 
% final z
% state_select = 3;
% A = 1;
% zf_lb = 0.25;
% zf_ub = 0.75;
% prog = prog.addStateConstraint(LinearConstraint(zf_lb,zf_ub,A),{N},state_select);

% final roll
state_select = 4;
A = 1;
rollf_lb = deg2rad(-5);
rollf_ub = deg2rad(5);
prog = prog.addStateConstraint(LinearConstraint(rollf_lb,rollf_ub,A),{N},state_select);

% final pitch
state_select = 5;
A = 1;
pitchf_lb = deg2rad(-5);
pitchf_ub = deg2rad(5);
prog = prog.addStateConstraint(LinearConstraint(pitchf_lb,pitchf_ub,A),{N},state_select);

% roll and pitch constraint
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
alt_lb = repmat(-1,N,1);
alt_ub = repmat(1,N,1);
prog = prog.addStateConstraint(LinearConstraint(alt_lb,alt_ub,A),{1:N},state_select);

% add constraint: obstacles
collision_constraint = generateConstraint(MinDistanceConstraint(r,0.05),0);
prog = prog.addStateConstraint(collision_constraint{1},1:N,1:getNumPositions(r));

% add constraint: goal position
goalConstraint = FunctionHandleConstraint([0;0;0],[0;0;0],14,@(x) final_state_con(r,x),1);
prog = prog.addStateConstraint(goalConstraint,{N});  % do we need a cell array here?

% add costs
% prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

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

function [f,df] = final_state_con(obj,x)
  goal_pos = [4;0;0.5];

  q = x(1:7);
  qd = x(8:14);
  kinsol = obj.doKinematics(q);
  [ball_pos,dBall_pos] = obj.forwardKin(kinsol,findFrameId(obj,'ball_com'),[0;0;-0.3]);

  f = ball_pos - goal_pos;
  df = [dBall_pos zeros(3,7)];
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
