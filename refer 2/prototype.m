%based on https://www.mathworks.com/matlabcentral/fileexchange/67926-portrait-drawing-using-computer-vision-and-robot-manipulator

clc;
clear;

I = imread('C:\Users\surya\OneDrive\_MS RAS AI\2 Fall 2022\MAE 547\Project\leo.jpg');

imshow(I)



[BW,thre] = edge(rgb2gray(I),'Canny',[0.0813 0.1281]); 
%figure, imshow(BW);

BW2 = bwmorph(BW,'skel',Inf);
%figure, imshow(BW2);

BW3 = bwmorph(BW2,'spur',3);
%figure, imshow(BW3);

branchPoints = bwmorph(BW3,'branch',1);
branchPoints = imdilate(branchPoints,strel('disk',1));
BW3 = BW3 & ~branchPoints;
%figure, imshow(BW3)

BWseg = bwareaopen(BW3,10);
%figure, imshow(BWseg)
[B,L] = bwboundaries(BWseg,'noholes');
%imshow(label2rgb(L, @jet, [.5 .5 .5]))
hold on
for k = 1:length(B)
   boundary = B{k};
   %plot(boundary(:,2), boundary(:,1), 'LineWidth', 3)
end

boundary = B{18};
edgeind = find(all(circshift(boundary,1)==circshift(boundary,-1),2),1)
boundary = circshift(boundary,-edgeind+1);
boundary = boundary(1:ceil(end/2),:)

for i = 1:length(B)
    boundary = B{i};
    edgeind = find(all(circshift(boundary,1)==circshift(boundary,-1),2),1);
    if ~isempty(edgeind)
        boundary = circshift(boundary,-edgeind+1);
        boundary = boundary(1:ceil(end/2),:);
    end
    B{i} = boundary;
end
%imshow(label2rgb(L, @jet, [.5 .5 .5]))
hold on
for k = 1:length(B)
   boundary = B{k};
   %plot(boundary(:,2), boundary(:,1), 'LineWidth', 3)
end

origin = [0 0]; % 画像の原点の位置
delta = 0.001; % 1ピクセルの長さ
B2 = B;
figure;
for i = 1:length(B)
    b = B{i};
    bx = -b(:,2)*delta+origin(1);
    by = b(:,1)*delta+origin(2);
    bz = zeros(length(bx),1);
    B2{i} = [bx by bz];
    %plot3(bx,by,bz); hold on;
end
grid on;

b_ = B2{1};

b=0.2*b_';



%traj = mstraj(b(:,2:end)', [0.5 0.5 0.5], [], b(:,1)',	0.02, 0.2);
%Tp=  SE3(traj) * SE3.oa( [0 1 0], [0 0 -1]);
%mdl_puma560

%q = p560.ikine6s(Tp);
% plotp(traj)
%p560.plot(q) 
figure;
L1 = 0.2;
L2 = -0.15;
L3 = -0.15;
L4 = -0.05;

robot = Drawing_arm(L1,L2,L3,L4);
Q = robot.homeConfiguration;
robot.show(Q,'preservePlot',false,'Frames','on');
set(gca,'CameraPosition',[7.6740 10.6196 11.3315],...
          'CameraTarget',[0.0292 -0.0476 0.0280],...
          'CameraUpVector',[0 0 1],'CameraViewAngle',1.3394,...
          'DataAspectRatio',[1 1 1],'Projection','perspective');


robot = Drawing_arm(L1,L2,L3,L4);
pen = robotics.RigidBody('pen');
penjnt = robotics.Joint('penjnt','fixed');
dhparam = [0 0 -0.09 0]; % DHパラメータ
setFixedTransform(penjnt,dhparam,'dh');
pen.Joint = penjnt;
addBody(robot,pen,'endeffector');
robot.show(Q,'preservePlot',false,'Frames','on','Parent',gca);
lineobj = findobj('Type','Line');
set(lineobj(1),'Visible','on');

set(gca,'CameraPosition',[7.6740 10.6196 11.3315],...
          'CameraTarget',[0.0292 -0.0476 0.0280],...
          'CameraUpVector',[0 0 1],'CameraViewAngle',1.3394,...
          'DataAspectRatio',[1 1 1],'Projection','perspective');
hold on;
figure(gcf);

for i = 1:length(B2)
    b = B2{i};
    plot3(b(:,1),b(:,2),b(:,3)); hold on;
end
figure(gcf);




for j=length(B2):-1:1
     temp = B2{j}
    b = temp;    
    
    tf = makehgtform('translate',b(1,:))
    

    plot3(b(:,1),b(:,2),b(:,3),'b','LineWidth',3);
    figure(gcf);    
    ik = robotics.InverseKinematics('RigidBodyTree',robot);
    [Q,~] = ik('pen',tf,[1 1 0 1 1 1],Q);
    bJoint = arrayfun(@(x) x.JointPosition,Q)
    robot.show(Q,'preservePlot',false,'Frames','off','Parent',gca);
    lineobj = findobj('Type','Line');
    set(lineobj(1),'Visible','on');
    figure(gcf);
    
    figure(gcf)
    temp2=0;
    for i=1:size(b,1)
      pose = [eye(3) b(i,:)';
          zeros(1,3) 1;];
      [Q,~] = ik('pen',pose,[1 1 0 1 1 1],Q);
      robot.show(Q,'preservePlot',false,'Frames','off','Parent',gca);
      lineobj = findobj('Type','Line');
      set(lineobj(1),'Visible','on');
      drawnow;
    end
end
