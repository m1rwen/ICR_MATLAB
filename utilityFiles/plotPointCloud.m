function hPlotCloud = plotPointCloud(ax, pointCloud, hPlotCloud, color, markerSize)
if nargin < 4, color = 'k'; end
if nargin < 5, markerSize = 8; end

if(size(pointCloud,2))
    % Plot points
    if isempty(hPlotCloud)
        hPlotCloud = scatter3(ax, pointCloud(1,:), pointCloud(2,:), pointCloud(3,:), markerSize, color, 'filled');
    else
        if ~ishandle(hPlotCloud)
            hPlotCloud = scatter3(ax, pointCloud(1,:), pointCloud(2,:), pointCloud(3,:), markerSize, color, 'filled');
        else
            set(hPlotCloud, 'XData', pointCloud(1,:),'YData', pointCloud(2,:), 'ZData', pointCloud(3,:));
        end
    end
else
    % Clear points
    if ~isempty(hPlotCloud)
        if ishandle(hPlotCloud)
            set(hPlotCloud, 'XData', [],'YData', [], 'ZData', []);
        end
    end
end

end