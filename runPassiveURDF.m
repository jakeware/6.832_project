function runPassiveURDF

p = PlanarRigidBodyManipulator('system.urdf');
ytraj = p.simulate([0 5],randn(2,1));

v = constructVisualizer(p);
v.playback(ytraj);
