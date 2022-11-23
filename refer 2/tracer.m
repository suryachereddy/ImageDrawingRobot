function [result,trace] = tracerrr(I)




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
B2 = B;
%figure;
for i = 1:length(B)
    b = B{i};
    bx = -b(:,2)*delta+origin(1);
    by = b(:,1)*delta+origin(2);
    bz = zeros(length(bx),1);
    B2{i} = [bx by bz];
    %plot3(bx,by,bz); hold on;
end
result = BWseg;
trace = B2;

