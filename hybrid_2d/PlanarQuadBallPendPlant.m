classdef PlanarQuadBallPendPlant < SecondOrderSystem

  % state:  
  %  q(1) - x load position
  %  q(2) - z load position
  %  q(3) - load roll angle (phi_l)
  %  q(4) - quad roll angle (phi_q)
  % input:
  %  u(1) - prop 1 thrust
  %  u(2) - prop 2 thrust

  properties  %  based on (Bouadi, Bouchoucha, Tadjine 2007)
    L = 0.25; % length of rotor arm
    m_q = 0.5; % mass of quadrotor
    m_l = 0.5; % mass of load
    I = 0.00383; % moment of inertia
    g = 9.81; % gravity
  end
  
  methods
    function obj = PlanarQuadBallPendPlant()
      obj = obj@SecondOrderSystem(3,2,true);
      obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
      % Implement the second-order dynamics
      c1 = ((u(1) + u(2))*cos(q(4) - q(3)) - obj.m_q*obj.L*qd(3)^2) / (obj.m_q + obj.m_l);
      qdd = [ 
        c1*sin(q(3));
        -c1*cos(q(3)) - obj.g;
        sin(q(4) - q(3))/obj.m_q/obj.L;
        obj.L/obj.I*(-u(1)+u(2))];
    end
  end
end
