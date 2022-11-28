function robot = Draw_Arm()

robot = importrobot('Draw_armV6.urdf','MeshPath',pwd);

% % Fix joint limits for the right gipper
% robot.Bodies{6}.Joint.PositionLimits = ...
%     [-robot.Bodies{6}.Joint.PositionLimits(2),...
%     robot.Bodies{6}.Joint.PositionLimits(1)];

% Add dummy link
dummy = robotics.RigidBody('dummy_link');
jnt1 = robotics.Joint('dummy_joint');
tform = trvec2tform([.05 0 0]); % User defined
setFixedTransform(jnt1,tform);
dummy.Joint = jnt1;
addBody(robot,dummy,'body4');

% % Add end effector
% body2 = robotics.RigidBody('end_effector');
% jnt2 = robotics.Joint('end_effector_joint');
% body2.Joint = jnt2;
% addBody(robot,body2,'dummy_link');

% Add virtual target position
dummy = robotics.RigidBody('target_link');
jnt1 = robotics.Joint('target_joint');
tform = trvec2tform([0 0 0]); % User defined
setFixedTransform(jnt1,tform);
dummy.Joint = jnt1;
addBody(robot,dummy,'dummy_link');

tip = robotics.RigidBody('tip');
tipjnt = robotics.Joint('tipjnt','fixed');
tform = trvec2tform([0 0 -0.09]);
setFixedTransform(tipjnt,tform);
tip.Joint = tipjnt;
addBody(robot,tip,'target_link');

%show(robot,randomConfiguration(robot));
show(robot)
end