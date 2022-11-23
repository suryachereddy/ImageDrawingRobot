function robot = Drawing_arm(L1, L2, L3, L4)
 
    body1 = rigidBody('body1');
    jnt1 = rigidBodyJoint('joint1', 'revolute');
    jnt1.HomePosition = 0;
    tform1 = trvec2tform([0 0 L1]);
    setFixedTransform(jnt1,tform1);
    body1.Joint = jnt1;
    
    robot = rigidBodyTree;
    
    addBody(robot,body1, 'base')
    
    body2 = rigidBody('body2');
    jnt2 = rigidBodyJoint('joint2', 'revolute');
    jnt2.HomePosition = 0;
    jnt2.JointAxis = [0 1 0];
    tform2 = trvec2tform([L2 0 0]);
    setFixedTransform(jnt2, tform2);
    body2.Joint = jnt2;
    
    addBody(robot,body2, 'body1')
    
    body3 = rigidBody('body3');
    jnt3 = rigidBodyJoint('joint3', 'revolute');
    jnt3.HomePosition = 0;
    jnt3.JointAxis = [0 1 0];
    tform3 = trvec2tform([L3 0 0]);
    setFixedTransform(jnt3, tform3);
    body3.Joint = jnt3;
    
    addBody(robot,body3, 'body2')
    
%     body4 = rigidBody('body4');
%     jnt4 = rigidBodyJoint('joint4', 'revolute');
%     jnt4.HomePosition = 0;
%     jnt4.JointAxis = [0 1 0];
%     tform4 = trvec2tform([L3 0 0]);
%     setFixedTransform(jnt4, tform4);
%     body4.Joint = jnt4;
%     
%     addBody(robot,body4, 'body3')
    

    bodyEndEffector = rigidBody('endeffector');
    tform5 = trvec2tform([L4 0 0]);
    setFixedTransform(bodyEndEffector.Joint,tform5);
    addBody(robot,bodyEndEffector, 'body3')

    body5 = rigidBody('Plate');
    jnt5 = rigidBodyJoint('joint5', 'fixed');
    tform5 = trvec2tform([0.075 0 -0.005]);
    setFixedTransform(jnt5, tform5);
    body5.Joint = jnt5;
    addBody(robot,body5, 'base')

end