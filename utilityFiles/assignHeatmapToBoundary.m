function [heatmapCartesian, xDim,yDim,zDim] = assignHeatmapToBoundary(heatmap,rangeVal,azimVal,elevVal,scene)
% Assign the index into zones
[~, xyzW, invRngAngIdx] = assignRadarCubeIndexPerZone(rangeVal, azimVal, elevVal, scene);
xyzW = xyzW(:,~invRngAngIdx).';
% figure; plot3(xyzW(:,1),xyzW(:,2),xyzW(:,3),'.'); daspect([1 1 1])
% xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

% Scene box is defined as x,y,z (left,near,bottom), width, depth, height
% Convert it to boundaryBox as x1,x2,y1,y2,z1,z2
boundaryBox = [scene.boundaryBox(1), scene.boundaryBox(1)+scene.boundaryBox(4), ...
    scene.boundaryBox(2), scene.boundaryBox(2)+scene.boundaryBox(5), ...
    scene.boundaryBox(3), scene.boundaryBox(3)+scene.boundaryBox(6)];

% Create the grid
xDim = boundaryBox(1):0.05:boundaryBox(2);
yDim = boundaryBox(3):0.05:boundaryBox(4);
zDim = boundaryBox(5):0.1:boundaryBox(6);
[Xq,Yq,Zq] = meshgrid(xDim, yDim, zDim);
% figure; plot3(Xq(:),Yq(:),Zq(:),'.'); daspect([1 1 1])
% xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

% Reshape the heatmap
heatmap = heatmap(:);
heatmap = heatmap(~invRngAngIdx);

% Remove duplicate data points
[xyzW, uqIdx] = unique(xyzW, 'rows');

% Grid the spherical data into Cartesian
heatmapCartesian = griddata(xyzW(:,1),xyzW(:,2),xyzW(:,3),heatmap(uqIdx),Xq,Yq,Zq);

% Reshape (Y-X-Z)
heatmapCartesian = reshape(heatmapCartesian, size(Xq));
% figure; meshLocal(xDim,yDim,max(heatmapCartesian,[],3)); daspect([1 1 1])
% xlabel('X(m)'); ylabel('Y(m)');
end