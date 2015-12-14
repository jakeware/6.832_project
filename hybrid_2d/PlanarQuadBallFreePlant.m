classdef PlanarQuadBallFreePlant < SecondOrderSystem

  % state:  
  %  q(1) - x quad position
  %  q(2) - z quad position
  %  q(3) - quad roll angle
  %  q(4) - x load position
  %  q(5) - z load position
  %  q(6) - load roll angle
  % input:
  %  u(1) - prop 1 thrust
  %  u(2) - prop 2 thrust

  properties  %  based on (Bouadi, Bouchoucha, Tadjine 2007)
    W = 0.25; % length of rotor arm
    L = 0.32; % length of pendulum
    m_q = 0.5; % mass of quadrotor
    m_l = 0.5; % mass of load
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
      f = u(1) + u(2);
      p = [sin(q(6));-cos(q(6))];
      e3 = [0;1];
      R = [
         cos(q(3)),-sin(q(3));
         sin(q(3)),cos(q(3))];
      x_ldd = -obj.g*e3;
      x_qdd = (f*R*e3)/obj.m_q - obj.g*e3;
      
      dx = q(4) - q(1);
      dz = q(5) - q(2);
      dxd = qd(4) - qd(1);
      dzd = qd(5) - qd(2);
      dxdd = x_ldd(1) - x_qdd(1);
      dzdd = x_ldd(2) - x_qdd(2);
      A = 1/(dx^2 + dz^2)^2;
      B = dz^2*(-2*dxd*dzd + dz*dxdd);
      C = dx^2*(2*dxd*dzd + dz*dxdd);
      D = dx^3*dzdd - dx*dz*(2*dxd^2 - 2*dzd^2 + dz*dzdd);
      
      qdd = [
        x_qdd(1);  % x quad position
        x_qdd(2);  % z quad position
        (u(2)-u(1))/obj.W/obj.I;  % quad roll angle (phi_q)
        x_ldd(1);  % x load position
        x_ldd(2);  % z load position
        A*(B + C - D)];  % load roll angle
    end
  end
end
