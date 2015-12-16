classdef HybridQuadBallPlant < HybridDrakeSystem

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
      obj = obj@HybridDrakeSystem(2,12);
      obj = setInputFrame(obj,CoordinateFrame('QBInput',2,'u',{'u1','u2'}));
      obj = setOutputFrame(obj,CoordinateFrame('QBOutput',12,'y',{'x_q','z_q','phi_q','x_l','z_l','phi_l','x_q_dot','z_q_dot','phi_q_dot','x_l_dot','z_l_dot','phi_l_dot'}));
      
      p1 = PlanarQuadBallPendPlant(obj);
      p2 = PlanarQuadBallFreePlant(obj);
      p3 = PlanarQuadBallPendPlant(obj);
      
      obj = setInputFrame(obj,p1.getInputFrame);
      obj = setOutputFrame(obj,p1.getOutputFrame);
      
      obj = obj.addMode(p1);
      obj = obj.addMode(p2);
      obj = obj.addMode(p3);

      obj = addTransition(obj,1,@pendGuard,@collisionDynamics,false,true);
      obj = addTransition(obj,2,@freeGuard,@collisionDynamics,false,true);
      obj = addTransition(obj,3,@pendGuard,@collisionDynamics,false,true);
      
      obj = setSimulinkParam(obj,'InitialStep','1e-3','MaxStep','0.05');
      obj = setInputLimits(obj,-50,50);
    end
    
    % pendulum to free
    function [g,dg] = freeGuard(obj,t,x,u)
      den = sqrt((x(1) - x(4))^2 - (x(2) - x(5))^2);
      g = den < obj.L;
      
      % gradient with respect to [t;x;u]
      dg = zeros(1,15);  
      dg(1) = 0;
      dg(2) = (x(1) - x(4)) / den;
      dg(3) = (x(2) - x(5)) / den;
      dg(4) = 0;
      dg(5) = -(x(1) - x(4)) / den;
      dg(6) = -(x(2) - x(5)) / den;
      dg(7:15) = 0;
    end
    
    % free to pendulum
    function [g,dg] = pendGuard(obj,t,x,u)
      den = sqrt((x(1) - x(4))^2 - (x(2) - x(5))^2);
      g = den >= obj.L;
      
      % gradient with respect to [t;x;u]
      dg = zeros(1,15);
      dg(1) = 0;
      dg(2) = (x(1) - x(4)) / den;
      dg(3) = (x(2) - x(5)) / den;
      dg(4) = 0;
      dg(5) = -(x(1) - x(4)) / den;
      dg(6) = -(x(2) - x(5)) / den;
      dg(7:15) = 0;
    end
      
    function [xp,mode,status,dxp] = collisionDynamics(obj,mode,t,xm,u)      
      % pendulum to free
      if (mode==1) 
        mode=2;
      
        % switching dynamcs
        xp = xm;
        
        % dxp gradient with respect to [mode,t,x,u];
        dxp = zeros(12,16);
        dxp(1:12,3:14) = eye(12);
      
      % free to pendulum
      else
        mode=1;
        
        % switching dynamics
        p = [sin(q(6));-cos(q(6))];
        xp = xm;
        xp(10:11) = xm(10:11) - dot(p,xm(10:11));
        
        % dxp gradient with respect to [mode,t,x,u];
        dxp = zeros(12,16);
        dxp(1:12,3:14) = eye(12);
        dxp(10,8) = -xp(10)*cos(xp(6));
        dxp(10,12) = 1 - sin(xp(6));
        dxp(11,8) = xp(11)*sin(xp(6));
        dxp(11,13) = 1 - cos(xp(6));
      end
      
      status = 0;
    end
    
  end
  
  methods (Static)
    function run()
      r = HybridQuadBallPlant();
      %v = CompassGaitVisualizer(r);
      traj = simulate(r,[0 10]);
      playback(v,traj);    
    end
  end  
  
end
