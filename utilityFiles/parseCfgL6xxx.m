function [P] = parseCfgL6xxx(cliCfg)
% For the units of each variable, please refer to the SDK user guide
P=[];
P.adcDataSource.isSourceFromFile = 0;
P.adcDataSource.numFrames = 0;

for k=1:length(cliCfg)
    C = strsplit(cliCfg{k});

    % Sensor Front-End Parameters
    if strcmp(C{1},'channelCfg')
        P.channelCfg.txChannelEn = str2double(C{3});
        P.dataPath.numTxAnt = bitand(bitshift(P.channelCfg.txChannelEn, 0),1) +...
            bitand(bitshift(P.channelCfg.txChannelEn,-1),1) + ...
            bitand(bitshift(P.channelCfg.txChannelEn,-2),1) + ...
            bitand(bitshift(P.channelCfg.txChannelEn,-3),1);
        P.channelCfg.rxChannelEn = str2double(C{2});
        P.dataPath.numRxAnt = bitand(bitshift(P.channelCfg.rxChannelEn,0),1) +...
            bitand(bitshift(P.channelCfg.rxChannelEn,-1),1) +...
            bitand(bitshift(P.channelCfg.rxChannelEn,-2),1) +...
            bitand(bitshift(P.channelCfg.rxChannelEn,-3),1);

    elseif strcmp(C{1},'chirpComnCfg')
        P.chirpComnCfg.digOutputSampRate  = 200 / str2double(C{2}); % in MHz
        P.chirpComnCfg.digOutputBitsSel   = str2double(C{3});
        P.chirpComnCfg.dfeFirSel          = str2double(C{4});
        P.chirpComnCfg.numAdcSamples      = str2double(C{5});
        P.chirpComnCfg.chirpTxMimoPatSel  = str2double(C{6});
        P.chirpComnCfg.chirpRampEndTime   = str2double(C{7}); % in us
        P.chirpComnCfg.chirpRxHpfSel      = str2double(C{8});

    elseif strcmp(C{1},'chirpTimingCfg')
        P.chirpTimingCfg.idleTime = str2double(C{2}); %in us
        P.chirpTimingCfg.numSkippedSamples = str2double(C{3}); %in adc samples
        P.chirpTimingCfg.ChirpTxStartTime = str2double(C{4}); %in us
        P.chirpTimingCfg.chirpSlope = str2double(C{5}); % in MHz/us
        P.chirpTimingCfg.startFreq = str2double(C{6});  % in GHz

    elseif strcmp(C{1},'frameCfg')
        P.frameCfg.numOfChirpsInBurst = str2double(C{2});
        P.frameCfg.numOfChirpsAccum = str2double(C{3});
        P.frameCfg.burstPeriodicity = str2double(C{4});
        P.frameCfg.numOfBurstsInFrame = str2double(C{5});
        P.frameCfg.framePeriodicity = str2double(C{6});
        P.frameCfg.numFrames = str2double(C{7});
        P.frameCfg.numLoops = P.frameCfg.numOfChirpsInBurst * P.frameCfg.numOfBurstsInFrame / P.dataPath.numTxAnt;
        P.frameCfg.chirpStartIdx = 0;
        P.frameCfg.chirpEndIdx = P.dataPath.numTxAnt - 1;
    
    elseif strcmp(C{1},'antGeometryCfg')
        dRxAntennaEn = [0 0];
        dRxAntennaElev = str2double(C{end-1});
        if (floor(dRxAntennaElev)~=dRxAntennaElev) % Not index
            dRxAntennaEn(1) = 1;
            P.frameCfg.dRxAntennaElev = dRxAntennaElev;
        end
        dRxAntennaAzim = str2double(C{end});
        if (floor(dRxAntennaAzim)~=dRxAntennaAzim) % Not index
            dRxAntennaEn(2) = 1;
            P.frameCfg.dRxAntennaAzim = dRxAntennaAzim;
        end
        dRxAntennaEn = all(dRxAntennaEn);

    elseif strcmp(C{1},'factoryCalibCfg')
        P.factoryCalibCfg.saveEnable = str2double(C{2});
        P.factoryCalibCfg.restoreEnable = str2double(C{3});
        P.factoryCalibCfg.rxGain = str2double(C{4});
        P.factoryCalibCfg.backoff = str2double(C{5});
        P.factoryCalibCfg.flashOffset = dec2hex(str2double(C{6}));

    
    % Detection Layer Parameters
    elseif strcmp(C{1},'sigProcChainCfg')
        P.dataPath.azimuthFftSize = str2double(C{2});
        P.dataPath.elevationFftSize = str2double(C{3});

    elseif strcmp(C{1},'guiMonitor')
        P.guiMonitor.pointCloud = str2double(C{2});
        P.guiMonitor.rangeProfile = str2double(C{3});
        P.guiMonitor.statsInfo = str2double(C{4});
        P.guiMonitor.temperatureInfo = str2double(C{5});
        P.guiMonitor.intrusionDetInfo = str2double(C{6});

    elseif strcmp(C{1},'dbgGuiMonitor')
        P.dbgGuiMonitor.detMat3D = str2double(C{2});
        P.dbgGuiMonitor.detSnr3D = str2double(C{3});
    
    elseif strcmp(C{1},'adcDataSource')
        if str2double(C{2}) == 1
            P.adcDataSource.isSourceFromFile = 1;
            [P.adcDataSource.adcDataFilePath, P.adcDataSource.adcDataFileName, adcDataFileNameExt] = fileparts(C{3});
            P.adcDataSource.adcDataFileName = [P.adcDataSource.adcDataFileName adcDataFileNameExt];
            [adcFid, errmsg] = fopen([P.adcDataSource.adcDataFilePath, '/', P.adcDataSource.adcDataFileName], 'rb');
            if ~isempty(errmsg)
                error(errmsg);
            end
            P.adcDataSource.numFrames = fread(adcFid, 1, 'int32');
            fclose(adcFid);
        else
            P.adcDataSource.isSourceFromFile = 0;
            P.adcDataSource.numFrames = 0;
        end

    elseif strcmp(C{1},'sensorPosition')
        P.sensorPosition.xOffset = str2double(C{2});
        P.sensorPosition.yOffset = str2double(C{3});
        P.sensorPosition.zOffset = str2double(C{4});
        P.sensorPosition.azimuthTilt = str2double(C{5});
        P.sensorPosition.elevationTilt = str2double(C{6});
        P.sensorPosition.yawTilt = 0;

    elseif strcmp(C{1},'occupancyBox')
        occBoxIdIntr = str2double(C{2}) + 1;
        P.occupancyBox.box(occBoxIdIntr,:) = str2double(C(3:8));

    elseif strcmp(C{1},'cuboidDef')
        occBoxId = str2double(C{2}) + 1;
        cuboidIdx = str2double(C{3}) + 1;
        % Set the existing format used in MATLAB-based processing
        P.zoneDef(occBoxId).cuboid(cuboidIdx).def = str2double(C(4:9));
        P.zoneDef(occBoxId).cuboid(cuboidIdx).x = str2double(C(4:5));
        P.zoneDef(occBoxId).cuboid(cuboidIdx).y = str2double(C(6:7));
        P.zoneDef(occBoxId).cuboid(cuboidIdx).z = str2double(C(8:9));

    elseif strcmp(C{1},'intruderDetCfg')
        P.intruderDetCfg.threshold = str2double(C{2});
        P.intruderDetCfg.free2activeThr = str2double(C{3});
        P.intruderDetCfg.active2freeThr = str2double(C{4});

    elseif strcmp(C{1},'intruderDetAdvCfg')
        occBoxId = str2double(C{2}) + 1;
        P.intruderDetAdvCfg.threshold(occBoxId) = str2double(C{3});
        P.intruderDetAdvCfg.free2activeThr(occBoxId) = str2double(C{4});
        P.intruderDetAdvCfg.active2freeThr(occBoxId) = str2double(C{5});

    elseif strcmp(C{1},'runningMode')
        P.runningMode = str2double(C{2});

    elseif strcmp(C{1},'featExtrCfg')
        P.featExtrCfg.maxNumPointsPerZonePerFrame = str2double(C{2});
        P.featExtrCfg.numFramesProc               = str2double(C{3});
        P.featExtrCfg.offsetCorrection            = str2double(C{4});
        P.featExtrCfg.dbScanFiltering             = str2double(C{5});
        P.featExtrCfg.dbScanEpsilon               = str2double(C{6});
        P.featExtrCfg.dbScanMinPts                = str2double(C{7});

    end
end


% Check the occupancy boxes
switch P.runningMode
    case {1,2}
        P.totNumZone = length(P.zoneDef);
        for z = 1:P.totNumZone
            P.zoneDef(z).numCuboids = length(P.zoneDef(z).cuboid);
            [zoneMinX, zoneMaxX] = bounds([P.zoneDef(z).cuboid.x]);
            [zoneMinY, zoneMaxY] = bounds([P.zoneDef(z).cuboid.y]);
            P.zoneDef(z).x_start = zoneMinX;
            P.zoneDef(z).x_len = zoneMaxX - zoneMinX;
            P.zoneDef(z).y_start = zoneMinY;
            P.zoneDef(z).y_len = zoneMaxY - zoneMinY;
        end

    otherwise
        % Assuming intruder, by default
        P.occupancyBox.numBoxes = occBoxIdIntr;
        assert(P.occupancyBox.numBoxes == size(P.occupancyBox.box,1), 'Occupancy box configuration error!');
end


% Set profileCfg
P.profileCfg.startFreq =  P.chirpTimingCfg.startFreq;
if P.frameCfg.numOfBurstsInFrame > 1
    P.profileCfg.idleTime(1) =  P.frameCfg.burstPeriodicity - ...
        P.frameCfg.numOfChirpsInBurst*(P.chirpTimingCfg.idleTime + P.chirpComnCfg.chirpRampEndTime) + ...
        P.chirpTimingCfg.idleTime;
    P.dataPath.chirpRepetitionPeriod = P.frameCfg.burstPeriodicity;
else
    P.profileCfg.idleTime(1) =  P.chirpTimingCfg.idleTime;
    P.dataPath.chirpRepetitionPeriod = (P.chirpTimingCfg.idleTime + P.chirpComnCfg.chirpRampEndTime) * P.dataPath.numTxAnt;
end
P.profileCfg.idleTime(2) =  P.chirpTimingCfg.idleTime;
P.profileCfg.adcStartTime =  P.chirpTimingCfg.numSkippedSamples / P.chirpComnCfg.digOutputSampRate;
P.profileCfg.rampEndTime = P.chirpComnCfg.chirpRampEndTime;
P.profileCfg.freqSlopeConst = P.chirpTimingCfg.chirpSlope;
P.profileCfg.numAdcSamples = P.chirpComnCfg.numAdcSamples;
P.profileCfg.digOutSampleRate = 1000 * P.chirpComnCfg.digOutputSampRate;


% Compute the data path parameters
P.dataPath.startFreq        = P.profileCfg.startFreq;
P.dataPath.adcStartTime     = P.profileCfg.adcStartTime;
P.dataPath.rampEndTime      = P.profileCfg.rampEndTime;
P.dataPath.freqSlopeConst   = P.profileCfg.freqSlopeConst;
P.dataPath.numAdcSamples    = P.profileCfg.numAdcSamples;
P.dataPath.digOutSampleRate = P.profileCfg.digOutSampleRate;


% Get the range parameters
P.dataPath.c = 3e8;
P.dataPath.numRangeBins = pow2roundup(P.dataPath.numAdcSamples)/2; % TODO: confirm real sampling from cfg
P.dataPath.rangeResolutionMeters = P.dataPath.c * P.dataPath.digOutSampleRate*1e3 /...
    (2 * P.dataPath.freqSlopeConst*1e12 * 2 * P.dataPath.numRangeBins);
P.dataPath.rangeVal = (0:P.dataPath.numRangeBins-1)*P.dataPath.rangeResolutionMeters;


% Get the angle parameters
if (P.runningMode == 0)
    if (dRxAntennaEn == 1)
        chirpLength = P.dataPath.numAdcSamples/(P.dataPath.digOutSampleRate*1e3);
        P.dataPath.carrierFrequency =   P.dataPath.startFreq*1e9 + (P.dataPath.adcStartTime*1e-6 + chirpLength/2)*P.dataPath.freqSlopeConst*1e12; % Hz center frequency
        lambda           =   P.dataPath.c/P.dataPath.carrierFrequency;
        d_over_lambda_el =   P.frameCfg.dRxAntennaElev*1e-3/lambda;
        d_over_lambda_az =   P.frameCfg.dRxAntennaAzim*1e-3/lambda;
    else
        d_over_lambda_el = 0.5;
        d_over_lambda_az = 0.5;
    end
    P.dataPath.azimVal =   (-P.dataPath.azimuthFftSize/2 : P.dataPath.azimuthFftSize/2-1) / d_over_lambda_az / P.dataPath.azimuthFftSize;
    P.dataPath.elevVal =   (-P.dataPath.elevationFftSize/2 : P.dataPath.elevationFftSize/2-1) / d_over_lambda_el / P.dataPath.elevationFftSize;
end


% Get the Doppler parameters
P.dataPath.numChirpsPerFrame = (P.frameCfg.chirpEndIdx - P.frameCfg.chirpStartIdx + 1) * P.frameCfg.numLoops;
P.dataPath.numDopplerBins = pow2roundup(P.dataPath.numChirpsPerFrame / P.dataPath.numTxAnt);

chirpRampTime           =   P.dataPath.numAdcSamples / (P.dataPath.digOutSampleRate*1e3);
carrierFrequency        =   P.dataPath.startFreq*1e9 + (P.dataPath.adcStartTime*1e-6 + chirpRampTime/2)*P.dataPath.freqSlopeConst*1e12; % Hz center frequency
P.dataPath.dopplerResolutionMps = P.dataPath.c / (2 * carrierFrequency *...
    P.dataPath.chirpRepetitionPeriod*1e-6 * P.dataPath.numDopplerBins);
end


% Utility function
function y = pow2roundup(x)
y = 2^ceil(log2(x));
end