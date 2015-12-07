classdef HybridPlant2 < HybridDrakeSystem
  
  methods 
    function obj = HybridPlant2()
      obj = obj@HybridDrakeSystem(4,16);
      p1 = QuadrotorBallPendPlant();
      p2 = QuadrotorBallFreePlant();
            
      obj = setInputFrame(obj,p1.getInputFrame);
      obj = setOutputFrame(obj,p1.getOutputFrame);
      obj = setInputFrame(obj,p2.getInputFrame);
      obj = setOutputFrame(obj,p2.getOutputFrame);
      
      obj = obj.addMode(p1);
      obj = obj.addMode(p2);

      obj = addTransition(obj,1,andGuards(obj,@lengthGuard1),@collisionDynamics,false,true);
      obj = addTransition(obj,2,andGuards(obj,@lengthGuard2),@collisionDynamics,false,true);
      
      obj = setSimulinkParam(obj,'InitialStep','1e-3','MaxStep','0.05');
      obj = setInputLimits(obj,-50,50);
    end
    
    function [g,dg] = lengthGuard1(obj,t,x,u)
      g = x(1)+x(2)+2;
      dg = [0,1,1,0,0,0];
    end
    
    function [g,dg] = lengthGuard2(obj,t,x,u)
      g = x(1)+x(2)+2;
      dg = [0,1,1,0,0,0];
    end
      
    function [xp,mode,status,dxp] = collisionDynamics(obj,mode,t,xm,u)
      m=obj.m; mh=obj.mh; a=obj.a; b=obj.b; l=obj.l;
      
      if (mode==1) mode=2;  % switch modes
      else mode=1; end
       
      xp = xm;
      
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
