function [zoneIdx, xyzLoc_W, invRngAngIdx] = assignRadarCubeIndexPerZone(rangeVal, azimVal, elevVal, scene, plotFlag, ax)
if nargin<5, plotFlag = 0; end
if nargin<6, ax = []; end

% Remove the invalid range bins
rangeVal(rangeVal<0) = NaN;

% Reshape each value into the same format of radar cube
rangeVal = reshape(rangeVal,[],1,1); % 1st dimension
azimVal  = reshape(azimVal,1,[],1); % 2nd dimension
elevVal  = reshape(elevVal,1,1,[]); % 3rd dimension

% Convert each point into the world coordinates (if the angles are already in radian)
% xLoc = rangeVal .* sin(azimVal) .*cos(elevVal);
% yLoc = rangeVal .* cos(azimVal) .*cos(elevVal);
% zLoc = rangeVal .* sin(elevVal);
% zLoc = repmat(zLoc,[1 length(azimVal) 1]);

% Convert each point into the world coordinates (based on the following conversion)
  % elevVal = asin(elevVal);                % in radian
  % azimVal = asin(azimVal/cos(elevVal));   % in radian
xLoc = rangeVal .* azimVal;
zLoc = rangeVal .* elevVal;
xLoc = repmat(xLoc,[1 1 length(elevVal)]);
zLoc = repmat(zLoc,[1 length(azimVal) 1]);
yLoc = sqrt(rangeVal.^2 -  xLoc.^2 - zLoc.^2);

% Define the invalid angles
invElevIdx = abs(elevVal) > 1;
tempElev = asin(elevVal);
invAzimIdx = abs(azimVal./cos(tempElev)) > 1;
invAngIdx = invElevIdx | invAzimIdx;
invAngIdx = repmat(invAngIdx,[length(rangeVal) 1 1]);

% Remove the invalid angle bins
xLoc(invAngIdx) = NaN;
yLoc(invAngIdx) = NaN;
zLoc(invAngIdx) = NaN;

% Extract all the invalid range-angle bins
assert(isequal(isnan(xLoc), isnan(yLoc)), 'Error in the invalid bins!');
assert(isequal(isnan(xLoc), isnan(zLoc)), 'Error in the invalid bins!');
invRngAngIdx = isnan(xLoc);
invRngAngIdx = invRngAngIdx(:).';

% Linearize the index
xyzLoc = [xLoc(:), yLoc(:), zLoc(:)].';

% Rotate and translate
xyzLoc_W = scene.Rot_TW*xyzLoc;
xyzLoc_W = xyzLoc_W + scene.sensorPos.';

% Get the x-y-z coordinates back
xLoc = xyzLoc_W(1,:);
yLoc = xyzLoc_W(2,:);
zLoc = xyzLoc_W(3,:);

% Scene box is defined as x,y,z (left,near,bottom), width, depth, height
% Convert it to boundaryBox as x1,x2,y1,y2,z1,z2
boundaryBox = [scene.boundaryBox(1), scene.boundaryBox(1)+scene.boundaryBox(4), ...
    scene.boundaryBox(2), scene.boundaryBox(2)+scene.boundaryBox(5), ...
    scene.boundaryBox(3), scene.boundaryBox(3)+scene.boundaryBox(6)];

% Assign index to zones
if plotFlag == 1
    if isempty(ax)
        figure; ax = gca;
    end
    hold on
    zoneIdx = assignIncabinZones(xLoc, yLoc, zLoc, boundaryBox, plotFlag, ax);
else
    zoneIdx = assignIncabinZones(xLoc, yLoc, zLoc, boundaryBox);
end

% Remove the invalid range-angle index
zoneIdx = zoneIdx & ~invRngAngIdx;

% Reshape it in terms of SNR (range - azimuth x elevation)
zoneIdx = reshape(zoneIdx, length(rangeVal), []);

end



% Utility function for the assignment
function zoneIdx = assignIncabinZones(xLoc, yLoc, zLoc, boundaryBox, plotFlag, ax)
if nargin<5, plotFlag = 0; end
if nargin<6, ax = []; end

% Initialize the zone Index
zoneIdx = zeros(size(xLoc),'uint8');

% driver side footwell
ind = plotCuboids(boundaryBox(1:2), boundaryBox(3:4), boundaryBox(5:6), xLoc, yLoc, zLoc, 'c', plotFlag, ax); %%original parameter
zoneIdx(ind) = 1;

end


% Utility function to plot cuboids
function ind = plotCuboids(x1, x2, x3, xLoc, yLoc, zLoc, color, plotFlag, ax)

if plotFlag 
    a = -pi : pi/2 : pi;                                % Define Corners
    ph = pi/4;        

    t1 = (x1(1) + x1(2))/2;  
    t2 = (x1(2) - x1(1))/2;  
    x = t1 + t2*[cos(a+ph); cos(a+ph)]/cos(ph);

    t1 = (x2(1) + x2(2))/2;  
    t2 = (x2(2) - x2(1))/2;  
    y = t1 + t2*[sin(a+ph); sin(a+ph)]/sin(ph);

    t1 = (x3(1) + x3(2))/2;  
    t2 = (x3(2) - x3(1))/2;  
    z = t1 + t2*[-ones(size(a)); ones(size(a))];
    surf(ax, x, z, y, 'FaceColor',color,'FaceAlpha',0.3); hold on;
end

ind = (xLoc > x1(1)) & ((xLoc < x1(2)));
ind = ind & (yLoc > x2(1)) & ((yLoc < x2(2)));
ind = ind & (zLoc > x3(1)) & ((zLoc < x3(2)));
end