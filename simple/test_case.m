function [utraj,xtraj,prog,r] = test_case

% initial and final state
start_pos = [0;0;0.5];
goal_pos = [2;0;0.5];

% time setup
N = 11;  
minimum_duration = .1;
maximum_duration = 4;

% setup
r = QuadrotorSimple();
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);  

% add constraint: initial state
x0 = Point(getStateFrame(r));  
x0.base_z = start_pos(3);
u0 = double(nominalThrust(r));
prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);

% add constraint: final state
xf = x0;                       
xf.base_x = goal_pos(1);
prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N);
prog = prog.addInputConstraint(ConstantConstraint(u0),N);

% initial trajectory
tf0 = 2;  % initial guess at time
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(u0);

% draw initial trajectory
v = constructVisualizer(r);
v.draw(0,double(x0));
prog = addPlanVisualizer(r,prog);

% add costs
prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

% solve for trajectory
tic
[xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
%[str,category] = snoptInfo(info)
toc

% create visualization
v.playback(xtraj,struct('slider',true));

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

