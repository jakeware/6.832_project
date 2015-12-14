classdef HybridQuadBallPlant < HybridDrakeSystem

% Questions:
% what is dg?
% what is dxp?

  properties
    W = 0.25; % length of rotor arm
    L = 0.32; % length of pendulum
    m_q = 0.5; % mass of quadrotor
    m_l = 0.5; % mass of load
    I = 0.00383; % moment of inertia
    g = 9.81; % gravity
  end

  methods 
    function obj = HybridQuadBallPlant()
      obj = obj@HybridDrakeSystem(2,6);
      p1 = QuadrotorBallPendPlant();
      p2 = QuadrotorBallFreePlant();
      
      obj = setInputFrame(obj,p1.getInputFrame);
      obj = setOutputFrame(obj,p1.getOutputFrame);
      
      obj = obj.addMode(p1);
      obj = obj.addMode(p2);

      obj = addTransition(obj,1,andGuards(obj,@lengthGuard1),@collisionDynamics,false,true);
      obj = addTransition(obj,2,andGuards(obj,@lengthGuard2),@collisionDynamics,false,true);
      
      obj = setSimulinkParam(obj,'InitialStep','1e-3','MaxStep','0.05');
      obj = setInputLimits(obj,-50,50);
    end
    
    function [g,dg] = lengthGuard1(obj,t,x,u)
      g = sqrt((x(1) - x(4)) - (x(2) - x(5))^2) < obj.L;
      dg = [0,1,1,0,0,0];  % what is this?
    end
    
    function [g,dg] = lengthGuard2(obj,t,x,u)
      g = sqrt((x(1) - x(4)) - (x(2) - x(5))^2) > obj.L;
      dg = [0,1,1,0,0,0];  % what is this?
    end
      
    function [xp,mode,status,dxp] = collisionDynamics(obj,mode,t,xm,u)      
      % pendulum to free
      if (mode==1) 
        mode=2;
      
        % switching dynamcs
        xp = xm;
      
      % free to pendulum
      else
        mode=1;
        
        % switching dynamics
        p = [sin(q(6));-cos(q(6))];
        xp(10:11) = xm(10:11) - dot(p,xm(10:11));
      end
      
      status = 0;
    end
    
  end
  
  methods (Static)
    function run()
      r = CompassGaitPlant();
      %v = CompassGaitVisualizer(r);
      traj = simulate(r,[0 10]);%,[2;0; 0; 2.0; -0.4]);
      playback(v,traj);    
    end
  end  
  
end
