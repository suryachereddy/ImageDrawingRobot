function robot = Draw_Arm()

robot = importrobot('Draw_armV8.urdf','MeshPath',pwd);

% Add dummy link
dummy = robotics.RigidBody('dummy_link');
jnt1 = robotics.Joint('dummy_joint');
tform = trvec2tform([.05 -0.01 0]);
setFixedTransform(jnt1,tform);
dummy.Joint = jnt1;
addBody(robot,dummy,'body4');

% Add virtual target position
dummy = robotics.RigidBody('target_link');
jnt1 = robotics.Joint('target_joint');
tform = trvec2tform([0 0 0]);
setFixedTransform(jnt1,tform);
dummy.Joint = jnt1;
addBody(robot,dummy,'dummy_link');

tip = robotics.RigidBody('tip');
tipjnt = robotics.Joint('tipjnt','fixed');
tform = trvec2tform([0 0 -0.09]);
setFixedTransform(tipjnt,tform);
tip.Joint = tipjnt;
addBody(robot,tip,'target_link');

show(robot,'Frames','off')
end