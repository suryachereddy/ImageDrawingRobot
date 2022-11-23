function robot = mikata_arm_4()
if ~exist('mikata_arm_4.urdf','file')
    disp('Downloading the URDF and STL files for Mikata Arm...');
    websave('mikata_arm_4.urdf','https://raw.githubusercontent.com/ROBOTIS-JAPAN-GIT/dynamixel_mikata_arm/master/mikata_arm_description/urdf/mikata_arm_4.urdf');
    websave('mikata_arm_4_gripper.stl','https://github.com/ROBOTIS-JAPAN-GIT/dynamixel_mikata_arm/raw/master/mikata_arm_description/meshes/mikata_arm_4_gripper.stl');
    websave('mikata_arm_4_joint1.stl','https://github.com/ROBOTIS-JAPAN-GIT/dynamixel_mikata_arm/raw/master/mikata_arm_description/meshes/mikata_arm_4_joint1.stl');
    websave('mikata_arm_4_joint2.stl','https://github.com/ROBOTIS-JAPAN-GIT/dynamixel_mikata_arm/raw/master/mikata_arm_description/meshes/mikata_arm_4_joint2.stl');
    websave('mikata_arm_4_joint3.stl','https://github.com/ROBOTIS-JAPAN-GIT/dynamixel_mikata_arm/raw/master/mikata_arm_description/meshes/mikata_arm_4_joint3.stl');
    websave('mikata_arm_4_joint4.stl','https://github.com/ROBOTIS-JAPAN-GIT/dynamixel_mikata_arm/raw/master/mikata_arm_description/meshes/mikata_arm_4_joint4.stl');
    websave('mikata_arm_4_joint5.stl','https://github.com/ROBOTIS-JAPAN-GIT/dynamixel_mikata_arm/raw/master/mikata_arm_description/meshes/mikata_arm_4_joint5.stl');
    disp('Done.');
end
robot = importrobot('mikata_arm_4.urdf','MeshPath',pwd);
% Fix joint limits for the right gipper
robot.Bodies{6}.Joint.PositionLimits = ...
    [-robot.Bodies{6}.Joint.PositionLimits(2),...
    robot.Bodies{6}.Joint.PositionLimits(1)];
% Add dummy link
body1 = robotics.RigidBody('dummy_link');
jnt1 = robotics.Joint('dummy_joint');
tform = trvec2tform([.04225 0 0]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
addBody(robot,body1,'link_5');
% Add end effector
body2 = robotics.RigidBody('end_effector');
jnt2 = robotics.Joint('end_effector_joint');
body2.Joint = jnt2;
addBody(robot,body2,'dummy_link');
% Add virtual target position
body1 = robotics.RigidBody('target_link');
jnt1 = robotics.Joint('target_joint');
tform = trvec2tform([.093 0 0]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
addBody(robot,body1,'dummy_link');




