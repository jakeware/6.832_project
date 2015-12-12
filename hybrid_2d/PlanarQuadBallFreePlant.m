classdef PlanarQuadBallFreePlant < SecondOrderSystem

  % state:  
  %  q(1) - x load position
  %  q(2) - z load position
  %  q(4) - x quad position
  %  q(5) - z quad position
  %  q(6) - quad roll angle
  % input:
  %  u(1) - prop 1 thrust
  %  u(2) - prop 2 thrust

  properties  %  based on (Bouadi, Bouchoucha, Tadjine 2007)
    L = 0.25; % length of rotor arm
    m = 0.486; % mass of quadrotor
    I = 0.00383; % moment of inertia
    g = 9.81; % gravity
  end
  
  methods
    function obj = PlanarQuadBallFreePlant()
      obj = obj@SecondOrderSystem(3,2,true);
      obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
      % Implement the second-order dynamics
      qdd = [
        0;
        -obj.g;
        -sin(q(3))/obj.m*(u(1)+u(2));
        -obj.g + cos(q(3))/obj.m*(u(1)+u(2));
        obj.L/obj.I*(-u(1)+u(2))];
    end
  end
end
