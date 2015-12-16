function [r,utraj,xtraj,z,traj_opt]=hybrid_ball_goal(goal_pos)
  %% NOTES
  %r.getStateFrame.getCoordinateNames  % print state variable names

  %% Setup
  start_pos = [0,1];
  
  t1 = 2;
  t2 = 2;
  t3 = 2;
  min_dur = .1;
  max_dur = t1 + t2 + t3;
  pend_length = 0.32;
  mode = [1;2;1];
  N = [5;5;5];
  duration = {[min_dur, max_dur],[min_dur, max_dur],[min_dur, max_dur]};

  %% FUNCTION
  r = HybridQuadBallPlant();
  prog = HybridTrajectoryOptimization(@DircolTrajectoryOptimization,r,mode,N,duration);  

  % add constraint: initial state
  x0 = zeros(12,1);
  x0(1) = 0;  % x_q
  x0(2) = 1;  % z_q
  x0(3) = 0;  % phi_q
  x0(4) = 0;  % x_l
  x0(5) = 1 - pend_length;  % z_l
  x0(6) = 0;  % phi_l
  u0 = [1;1];

  % quad bounding box
  quad_bound_min = [-2,pend_length];
  quad_bound_max = [2,3];

  % plan visualization
%   prog = prog.addModeStateConstraint(1,ConstantConstraint(double(x0)),1);
%   prog = prog.addModeInputConstraint(1,ConstantConstraint(u0),1);

  % add constraint: quad x-position
%   state_select = 2;
%   A = eye(N);
%   alt_lb = repmat(quad_bound_min(1),N,1);
%   alt_ub = repmat(quad_bound_max(1),N,1);
%   prog = prog.addModeStateConstraint(LinearConstraint(alt_lb,alt_ub,A),{1:N},state_select);

  % add constraint: quad z-position
%   state_select = 3;
%   A = eye(N);
%   alt_lb = repmat(quad_bound_min(2),N,1);
%   alt_ub = repmat(quad_bound_max(2),N,1);
%   prog = prog.addStateConstraint(LinearConstraint(alt_lb,alt_ub,A),{1:N},state_select);

  % add constraint: final x-position
  state_select = 1;
  A = 1;
  alt_lb = goal_pos(1) - 0.1;
  alt_ub = goal_pos(1) + 0.1;
  prog = prog.addModeStateConstraint(3,LinearConstraint(alt_lb,alt_ub,A),N(3),state_select);
  
  % add constraint: final z-position
  state_select = 2;
  A = 1;
  alt_lb = goal_pos(2) - 0.1;
  alt_ub = goal_pos(2) + 0.1;
  prog = prog.addModeStateConstraint(3,LinearConstraint(alt_lb,alt_ub,A),N(3),state_select);
 
%   % add constraint: goal position
%   goalConstraint1 = FunctionHandleConstraint([0;0;0],[0;0;0],r.getNumStates,@(x) top_state_con(r,x,top_goal_pos,link_length),1);
%   prog = prog.addStateConstraint(goalConstraint1,{N_top});
% 
%   % add constraint: goal position
%   goalConstraint2 = FunctionHandleConstraint([0;0;0],[0;0;0],r.getNumStates,@(x) fwd_state_con(r,x,fwd_goal_pos,link_length),1);
%   prog = prog.addStateConstraint(goalConstraint2,{N_top + 6});
% 
%   % add costs
%   prog = prog.addRunningCost(@cost);
%   % prog = prog.addFinalCost(@finalCost);
% 
  % initial trajectory
  x1 = x0;
  x1(1) = start_pos(1);
  x1(2) = start_pos(2);
  
  x2 = x0;
  x2(1) = 0.3*goal_pos(1);
  x2(2) = 0.3*goal_pos(2);
  
  x3 = x0;
  x3(1) = 0.6*goal_pos(1);
  x3(2) = 0.6*goal_pos(2);
  
  traj_init{1}.x0 = x1;
  traj_init{2}.x0 = x2;
  traj_init{3}.x0 = x3;
 
  t_init{1} = linspace(0,t1,N(1));
  t_init{2} = linspace(0,t2,N(2));
  t_init{3} = linspace(0,t3,N(3));
  
  % compile
  prog = prog.compile();

  % solve for trajectory
  tic
  [xtraj,utraj,z,F,info] = prog.solveTraj(t_init,traj_init);
  [str,category] = snoptInfo(info)
  toc
end

% % cost function
% function [g,dg] = cost(dt,x,u)
%   R = eye(4);
%   g = u'*R*u;
%   dg = [zeros(1,1+size(x,1)),2*u'*R];
% end
% 
% % final cost function
% function [h,dh] = finalCost(t,x)
%   h = t;
%   dh = [1,zeros(1,size(x,1))];
% end