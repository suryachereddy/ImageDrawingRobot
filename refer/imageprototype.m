clear; clc;
I = imread('leo.jpg');
I=imresize(I,[32,32]);
I = rgb2gray(I);
[c,h]=imcontour(I,2);

B=c;
path = [ 0.1*B; zeros(1,numcols(B))];
k = find(isnan(path(1,:)));
path(:,k) = path(:,k-1); path(3,k) = 0.2;
traj = mstraj(path(:,2:end)', [0.5 0.5 0.5], [], path(:,1)',	0.02, 0.2);
plot3(traj(:,1), traj(:,2), traj(:,3))

