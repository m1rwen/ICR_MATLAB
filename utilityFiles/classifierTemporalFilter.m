% Temporal filtered tags
function [scoreOcc, tagCurrent, tagTemporal] = classifierTemporalFilter(score, classifierConfidenceScore, minNumOccupancyHits, tagTemporal)
classLabels = 0:1; % Two classes by default
tagTemporalFiltSize = size(tagTemporal,2);

gIdx = find(score>classifierConfidenceScore);
if isempty(gIdx)
    tag = 0;
else
    tag = classLabels(gIdx);
end
scoreOcc = score(2); % Second one is the occupancy score

% Update the tag buffer
if tagTemporalFiltSize>1
    tagTemporal = [tagTemporal(2:end), tag];
else
    tagTemporal = tag;
end

% Update the decision status of current frame
if sum(tagTemporal) > minNumOccupancyHits
    tagCurrent = 1;
else
    tagCurrent = 0;
end
end