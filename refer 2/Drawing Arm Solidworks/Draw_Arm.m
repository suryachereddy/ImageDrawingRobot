function robot = Draw_Arm()

robot = importrobot('Draw_armV3.urdf','MeshPath',pwd);

% % Fix joint limits for the right gipper
% robot.Bodies{6}.Joint.PositionLimits = ...
%     [-robot.Bodies{6}.Joint.PositionLimits(2),...
%     robot.Bodies{6}.Joint.PositionLimits(1)];

% Add dummy link
body1 = robotics.RigidBody('dummy_link');
jnt1 = robotics.Joint('dummy_joint');
tform = trvec2tform([.05 0 0]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
addBody(robot,body1,'body4');

% % Add end effector
% body2 = robotics.RigidBody('end_effector');
% jnt2 = robotics.Joint('end_effector_joint');
% body2.Joint = jnt2;
% addBody(robot,body2,'dummy_link');

% Add virtual target position
body1 = robotics.RigidBody('target_link');
jnt1 = robotics.Joint('target_joint');
tform = trvec2tform([0 0 0]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
addBody(robot,body1,'dummy_link');

show(robot)
end