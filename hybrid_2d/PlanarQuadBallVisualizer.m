classdef PlanarQuadBallVisualizer < Visualizer
% Implements the draw function for the Planar Quadrotor model

  properties
    W=.25;  % moment arm
  end

  methods
    function obj = PlanarQuadBallVisualizer(plant)
      typecheck(plant,'PlanarQuadBallPendPlant');
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.W=plant.W;
    end
    
    function draw(obj,t,x)
      % Draw the quadrotor.  
      persistent hFig base pin prop;

      if (isempty(hFig))
        hFig = sfigure(25);
        set(hFig,'DoubleBuffer', 'on');
        
        base = [1.2*obj.W*[1 -1 -1 1]; .025*[1 1 -1 -1]];
        pin = [.005*[1 1 -1 -1]; .1*[1 0 0 1]];
        a = linspace(0,2*pi,50);
        prop = [obj.W/1.5*cos(a);.1+.02*sin(2*a)];
      end
            
      sfigure(hFig); cla; hold on; view(0,90);
      
      r = [cos(x(3)), -sin(x(3)); sin(x(3)), cos(x(3))];
      
      p = r*base;
      patch(x(1)+p(1,:), x(2)+p(2,:),1+0*p(1,:),'b','FaceColor',[.6 .6 .6])
      
      p = r*[obj.W+pin(1,:);pin(2,:)];
      patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 0]);
      p = r*[-obj.W+pin(1,:);pin(2,:)];
      patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 0]);
      
      p = r*[obj.W+prop(1,:);prop(2,:)];
      patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 1]);
      p = r*[-obj.W+prop(1,:);prop(2,:)];
      patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 1]);
      
      title(['t = ', num2str(t(1),'%.2f') ' sec']);
      set(gca,'XTick',[],'YTick',[])
      
      axis image; axis([-2.0 2.0 -1.0 1.0]);
      drawnow;
    end    
  end
  
end
