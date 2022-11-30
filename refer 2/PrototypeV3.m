clear; clc;

I = imread('C:\Users\surya\OneDrive\_MS RAS AI\2 Fall 2022\MAE 547\Project\leo.jpg'); % your path here
%I = imread('C:\Users\jason\Documents\MATLAB\MAE547\Project2\leo.jpg');
imshow(I)


[BW,thre] = edge(im2gray(I),'Canny',[0.0813 0.1281]); % CHANGED TO im2gray

BW2 = bwmorph(BW,'skel',Inf);

BW3 = bwmorph(BW2,'spur',3);

branchPoints = bwmorph(BW3,'branch',1);
branchPoints = imdilate(branchPoints,strel('disk',1));
BW3 = BW3 & ~branchPoints;

BWseg = bwareaopen(BW3,10);
figure, imshow(BWseg)
[B,L] = bwboundaries(BWseg,'noholes');
hold on
for k = 1:length(B)
   boundary = B{k};
end

boundary = B{end}; % CHANGED TO end
edgeind = find(all(circshift(boundary,1)==circshift(boundary,-1),2),1);
boundary = circshift(boundary,-edgeind+1);
boundary = boundary(1:ceil(end/2),:);

for i = 1:length(B)
    boundary = B{i};
    edgeind = find(all(circshift(boundary,1)==circshift(boundary,-1),2),1);
    if ~isempty(edgeind)
        boundary = circshift(boundary,-edgeind+1);
        boundary = boundary(1:ceil(end/2),:);
    end
    B{i} = boundary;
end

hold on
for k = 1:length(B)
   boundary = B{k};
end


origin = [0.1 0]; 
delta = 0.001; 
B2 = B;
figure;
for i = 1:length(B)
    b = B{i};
    bx = b(:,2)*delta+origin(1);
    by = -b(:,1)*delta+origin(2);
    bz = zeros(length(bx),1);
    B2{i} = [bx by bz];
    plot3(bx,by,bz); hold on;
end
grid on;



figure;



robot = Draw_Arm;
Q = robot.homeConfiguration;
robot.show(Q,'preservePlot',false,'Frames','off','Parent',gca,'FastUpdate',1);
lineobj = findobj('Type','Line');
set(lineobj(1),'Visible','on');

set(gca,'CameraPosition',[7.6740 -10.6196 12],...
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

for j=1:length(B2)
    temp = B2{j};
    b = temp;    
    
    
    firstpoint = b(1, :)
    if j>1
        xs = lastpoint(1);
        ys = lastpoint(2);
        zs = lastpoint(3);
        xl = firstpoint(1);
        yl = firstpoint(2);
        zl = firstpoint(3);
        n =20;
        lift =0.01;
        p = [linspace(xs,xl,n)' linspace(ys, yl, n)' [linspace(zs,lift, n/2) linspace(lift, zl, n/2)]']
       for i=1:size(p,1)
      pose = [eye(3) p(i,:)';
          zeros(1,3) 1;];
      [Q,~] = ik('tip',pose,[1 1 0 1 1 1],Q);
      robot.show(Q,'preservePlot',false,'Frames','off','Parent',gca,'FastUpdate',1);
      lineobj = findobj('Type','Line');
      set(lineobj(1),'Visible','on');
      drawnow;
      
       end 
       plot3(b(:,1),b(:,2),b(:,3),'b','LineWidth',1.75);
       lineobj = findobj('Type','Line');
    set(lineobj(1),'Visible','on');
       figure(gcf);
    else
    tf = makehgtform('translate',b(1,:));
    plot3(b(:,1),b(:,2),b(:,3),'b','LineWidth',1.75);
    figure(gcf);    
    ik = robotics.InverseKinematics('RigidBodyTree',robot);
    [Q,~] = ik('tip',tf,[1 1 0 1 1 1],Q);
    bJoint = arrayfun(@(x) x.JointPosition,Q);
    robot.show(Q,'preservePlot',false,'Frames','off','Parent',gca,'FastUpdate',1);
    lineobj = findobj('Type','Line');
    set(lineobj(1),'Visible','on');
    figure(gcf);
    end
    figure(gcf)
    temp2=0;
    for i=1:size(b,1)
      pose = [eye(3) b(i,:)';
          zeros(1,3) 1;];
      [Q,~] = ik('tip',pose,[1 1 0 1 1 1],Q);
      robot.show(Q,'preservePlot',false,'Frames','off','Parent',gca,'FastUpdate',1);
      lineobj = findobj('Type','Line');
      set(lineobj(1),'Visible','on');
      drawnow;
      lastpoint = b(i,:);
    end
end