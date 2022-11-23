clc;
clear;
load hershey
B = hershey{'B'}
B.stroke
path = [ 0.4*B.stroke; zeros(1,numcols(B.stroke))];
k = find(isnan(path(1,:)));
path(:,k) = path(:,k-1); path(3,k) = 0.2;
traj = mstraj(path(:,2:end)', [0.5 0.5 0.5], [], path(:,1)',	0.02, 0.2);
plot3(traj(:,1), traj(:,2), traj(:,3))
 
hold on;
plot3(traj(:,1), traj(:,2), traj(:,3),Color='r')
hold off;

Tp = SE3(0.5, 0, 0) * SE3(traj) * SE3.oa( [0 1 0], [0 0 -1]);
trplot(Tp(1),length=0.01, notext=true)
hold on;
for i=2:length(Tp)
    trplot(Tp(i),length=0.01, notext=true)
end

mdl_puma560

q = p560.ikine6s(Tp);

p560.plot(q) 
