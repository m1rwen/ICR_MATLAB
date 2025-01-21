function [fHist, targetFrameNum, pointCloudIn, point3D_W, intrusionDetInfo, classifierInfo] = parseDataFromDevice(hDataSerialPort, fHist, frameNum, targetFrameNum, sensorMount, frameStart)

% Initialize the parameters
pointCloudIn = single(zeros(5,0));
point3D_W = single(zeros(3,0));
intrusionDetInfo = [];
classifierInfo = [];
rngProfile = [];


% Defined TLV's
MMWDEMO_OUTPUT_MSG_RANGE_PROFILE                        = 2;

MMWDEMO_OUTPUT_MSG_STATS                                = 6;

MMWDEMO_OUTPUT_MSG_INTRUSION_DET_INFO                   = 12;
MMWDEMO_OUTPUT_MSG_INTRUSION_DET_3D_DET_MAT             = 13;
MMWDEMO_OUTPUT_MSG_INTRUSION_DET_3D_SNR                 = 14;

MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS                  = 301;
MMWDEMO_OUTPUT_MSG_POINT_CLOUD                          = 1020;

MMWDEMO_OUTPUT_MSG_OCCUPANCY_FEATURES                   = 1040;
MMWDEMO_OUTPUT_MSG_OCCUPANCY_CLASSIFICATION_RES         = 1041;
MMWDEMO_OUTPUT_MSG_OCCUPANCY_HEIGHT_RES                 = 1042;

MMWDEMO_OUTPUT_DEBUG_ANT_GEOMETRY                       = 2000;
MMWDEMO_OUTPUT_DEBUG_DET_MAT_ANGLE_SLICE                = 2001;
MMWDEMO_OUTPUT_DEBUG_SNR_MAT_ANGLE_SLICE                = 2002;

MMWDEMO_OUTPUT_DEBUG_CAPON_HEATMAP                      = 2003;
MMWDEMO_OUTPUT_DEBUG_CAPON_RAW_CFAR_POINT_CLOUD         = 2004;


% Header data
syncPatternUINT64 = typecast(uint16([hex2dec('0102'),hex2dec('0304'),hex2dec('0506'),hex2dec('0708')]),'uint64');
syncPatternUINT8 = typecast(uint16([hex2dec('0102'),hex2dec('0304'),hex2dec('0506'),hex2dec('0708')]),'uint8');

frameHeaderStructType = struct(...
    'sync',             {'uint64', 8}, ... % See syncPatternUINT64 below
    'version',          {'uint32', 4}, ...
    'packetLength',     {'uint32', 4}, ... % In bytes, including header
    'platform',         {'uint32', 4}, ...
    'frameNumber',      {'uint32', 4}, ... % Starting from 0
    'timeCPUCycles',    {'uint32', 4}, ... % Time in CPU cycles when the message was created
    'numDetectedObj',   {'uint32', 4}, ...
    'numTLVs' ,         {'uint32', 4}, ... % Number of TLVs in thins frame
    'subFrameNum',      {'uint32', 4});    % 0 if advanced subframe mode is disabled, otherwise 0 to N

tlvHeaderStruct = struct(...
    'type',             {'uint32', 4}, ... % TLV object Type
    'length',           {'uint32', 4});    % TLV object Length, in bytes, not including TLV header

% Point Cloud TLV reporting unit for all reported points in spherical coordinates
pointUintStruct = struct(...
    'elevUnit',             {'float', 4}, ... % elevation, in rad
    'azimUnit',             {'float', 4}, ... % azimuth, in rad
    'dopplerUnit',          {'float', 4}, ... % Doplper, in m/s
    'rangeUnit',            {'float', 4}, ... % Range, in m
    'snrUnit',              {'float', 4});    % SNR, ratio

% Point Cloud TLV object consists of an array of points in spherical coordinates.
% Each point has a structure defined below
pointStruct = struct(...
    'elevation',        {'int8', 1}, ...    % elevation, in rad
    'azimuth',          {'int8', 1}, ...    % azimuth, in rad
    'doppler',          {'int16', 2}, ...   % Doppler, in m/s
    'range',            {'uint16', 2}, ...  % Range, in m
    'snr',              {'uint16', 2});     % SNR, ratio

% Point Cloud TLV reporting unit for all reported points in cartesian coordinates
pointUintStructCartesian = struct(...
    'xyzUnit',              {'float', 4}, ... % x/y/z in m
    'dopplerUnit',          {'float', 4}, ... % Doplper, in m/s
    'snrUnit',              {'float', 4}, ... % SNR, in dB
    'noiseUnit',            {'float', 4}, ... % noise, in dB
    'numDetPointsMajor',    {'uint16', 2}, ... % number of detected points in Major mode
    'numDetPointsMinor',    {'uint16', 2});    % number of detected points in Minor mode

% Point Cloud TLV object consists of an array of points in cartesian coordinates.
% Each point has a structure defined below
pointStructCartesian = struct(...
    'x',                    {'int16', 2}, ... % x in m
    'y',                    {'int16', 2}, ... % y in m
    'z',                    {'int16', 2}, ... % z in m
    'doppler',              {'int16', 2}, ... % Doplper, in m/s
    'snr',                  {'uint8', 1},...    % SNR ratio in 0.1dB
    'noise',                {'uint8', 1});    % type 0-major motion, 1-minor motion

frameHeaderLengthInBytes = lengthFromStruct(frameHeaderStructType);
tlvHeaderLengthInBytes = lengthFromStruct(tlvHeaderStruct);
pointLengthInBytes = lengthFromStruct(pointStruct);
pointUnitLengthInBytes = lengthFromStruct(pointUintStruct);
pointLengthCartesianInBytes = lengthFromStruct(pointStructCartesian);
pointUnitLengthCartesianInBytes = lengthFromStruct(pointUintStructCartesian);

% Start processing frame
gotHeader = 0;
lostSync = 0;
frameParsed = 0;

while (1)
    while (frameParsed == 0)
        fHist(frameNum).timestamp = 1000*toc(frameStart);

        if(gotHeader == 0)
            % Read the header first
            [rxHeader, byteCount, outOfSyncBytes] = readFrameHeader(hDataSerialPort, frameHeaderLengthInBytes, syncPatternUINT8);
            gotHeader = 1;
        end

        % Double check the header size
        if(byteCount ~= frameHeaderLengthInBytes)
            reason = 'Header Size is wrong';
            lostSync = 1;
            break;
        end

        % Double check the sync pattern
        magicBytes = typecast(uint8(rxHeader(1:8)), 'uint64');
        if(magicBytes ~= syncPatternUINT64)
            reason = 'No SYNC pattern';
            lostSync = 1;
            break;
        end

        % Header is read, start the frame timer
        fHist(frameNum).start = 1000*toc(frameStart);

        % Update the bytes available
        bytesAvailable = hDataSerialPort.NumBytesAvailable;
        fHist(frameNum).bytesAvailable = bytesAvailable;

        % parse the header
        frameHeader = readToStruct(frameHeaderStructType, rxHeader);
        if(gotHeader == 1)
            if(frameHeader.frameNumber >= targetFrameNum)
                % We have a valid header
                targetFrameNum = frameHeader.frameNumber;
                if outOfSyncBytes > 0
                    disp(['Found sync at frame ',num2str(targetFrameNum),'(',num2str(frameNum),'). ', 'Discarded out of sync bytes: ' num2str(outOfSyncBytes)]);
                end
                gotHeader = 0;
            else
                reason = 'Old Frame';
                gotHeader = 0;
                lostSync = 1;
                break;
            end
        end

        % Start processing the header
        fHist(frameNum).targetFrameNum = targetFrameNum;
        fHist(frameNum).header = frameHeader;

        dataLength = frameHeader.packetLength - frameHeaderLengthInBytes;
        fHist(frameNum).bytes = dataLength;

        if(dataLength > 0)
            % Read all packet
            [rxData, byteCount] = readSerial(hDataSerialPort, double(dataLength), 'uint8');
            if(byteCount ~= double(dataLength))
                reason = 'Data Size is wrong';
                lostSync = 1;
                break;
            end
            offset = 0;

            % TLV Parsing
            for nTlv = 1:frameHeader.numTLVs
                tlvType = typecast(uint8(rxData(offset+1:offset+4)), 'uint32');
                tlvLength = typecast(uint8(rxData(offset+5:offset+8)), 'uint32');
                if(tlvHeaderLengthInBytes + tlvLength + offset > dataLength)
                    reason = 'TLV Size is wrong';
                    lostSync = 1;
                    break;
                end
                offset = offset + tlvHeaderLengthInBytes;
                valueLength = tlvLength;

                switch(tlvType)

                    case MMWDEMO_OUTPUT_MSG_RANGE_PROFILE
                        rngProfile = double(typecast(uint8(rxData(offset+1: offset+valueLength)),'uint32'));
                        offset = offset + valueLength;

                    case MMWDEMO_OUTPUT_MSG_STATS
                        msgTemp = double(typecast(uint8(rxData(offset+1: offset+valueLength)),'uint32'));
                        sensorTimingInfo.interFrameProcessingTime_usec = msgTemp(1);
                        sensorTimingInfo.transmitOutputTime_usec = msgTemp(2);
                        if length(msgTemp) == 7
                            sensorTimingInfo.dspProcessingTime_usec = msgTemp(3);
                        else
                            sensorTimingInfo.dspProcessingTime_usec = 0;
                        end
                        offset = offset + valueLength;

                    case MMWDEMO_OUTPUT_MSG_INTRUSION_DET_INFO
                        offsetTmp = offset;
                        intrusionDetInfo.numOccBoxes = double(typecast(uint8(rxData(offsetTmp+1:offsetTmp+4)),'int32'));
                        offsetTmp = offsetTmp + 4;
                        intrusionDetInfo.occBoxSignal = double(typecast(uint8(rxData(offsetTmp+1:offsetTmp+intrusionDetInfo.numOccBoxes*4)),'single'));
                        offsetTmp = offsetTmp + 4*intrusionDetInfo.numOccBoxes;
                        intrusionDetInfo.occBoxDecision = double(typecast(uint8(rxData(offsetTmp+1:offsetTmp+intrusionDetInfo.numOccBoxes*1)),'uint8'));
                        offsetTmp = offsetTmp + intrusionDetInfo.numOccBoxes;
                        assert(offset+valueLength == offsetTmp, 'Some data is missing and not read in this TLV!');
                        offset = offsetTmp;

                    case MMWDEMO_OUTPUT_MSG_INTRUSION_DET_3D_DET_MAT
                        intrusionDetInfo.detMat3D = double(typecast(uint8(rxData(offset+1:offset+valueLength)),'uint32'));
                        offset = offset + valueLength;

                    case MMWDEMO_OUTPUT_MSG_INTRUSION_DET_3D_SNR
                        detSnr3DdB = double(typecast(uint8(rxData(offset+1:offset+valueLength)),'int16'));
                        intrusionDetInfo.detSnr3D = 2.^(detSnr3DdB/2048);
                        offset = offset + valueLength;

                    case MMWDEMO_OUTPUT_MSG_OCCUPANCY_FEATURES
                        classifierInfo.occFeatures = double(typecast(uint8(rxData(offset+1: offset+valueLength)),'single'));
                        offset = offset + valueLength;
                    
                    case MMWDEMO_OUTPUT_MSG_OCCUPANCY_CLASSIFICATION_RES
                        classifierInfo.occPredictions = double(typecast(uint8(rxData(offset+1: offset+valueLength)),'uint8')) / 128; % Predictions received in Q7 format
                        offset = offset + valueLength;
                    
                    case MMWDEMO_OUTPUT_MSG_OCCUPANCY_HEIGHT_RES
                        classifierInfo.occHeight = double(typecast(uint8(rxData(offset+1: offset+valueLength)),'single'));
                        offset = offset + valueLength;

                    case MMWDEMO_OUTPUT_DEBUG_ANT_GEOMETRY
                        antSymb = double(typecast(uint8(rxData(offset+1:offset+valueLength)),'int16'));
                        offset = offset + valueLength;
                        antSymb = antSymb(2:2:end) + 1j*antSymb(1:2:end);

                    case MMWDEMO_OUTPUT_DEBUG_DET_MAT_ANGLE_SLICE
                        detMatSlice = double(typecast(uint8(rxData(offset+1:offset+valueLength)),'uint32'));
                        offset = offset + valueLength;
                        detMatSlice = reshape(detMatSlice, 16,16);
                        detMatSlice = detMatSlice';

                    case MMWDEMO_OUTPUT_DEBUG_SNR_MAT_ANGLE_SLICE
                        snrMatSlice = double(typecast(uint8(rxData(offset+1:offset+valueLength)),'int16'));
                        offset = offset + valueLength;
                        snrMatSlice = reshape(snrMatSlice, 16,16);
                        snrMatSlice = snrMatSlice';

                    case MMWDEMO_OUTPUT_DEBUG_CAPON_HEATMAP
                        dimension = double(typecast(uint8(rxData(offset+1:offset+3*4)),'int32'));
                        offset = offset + 3*4;
                        valueLength = valueLength - 3*4;
                        caponHeatMap = double(typecast(uint8(rxData(offset+1:offset+valueLength)),'single'));
                        caponHeatMap = reshape(caponHeatMap, dimension(1), dimension(2), dimension(3));
                        offset = offset + valueLength;
                        clear dimension

                    case MMWDEMO_OUTPUT_DEBUG_CAPON_RAW_CFAR_POINT_CLOUD
                        caponCfarNumDetPoints = double(typecast(uint8(rxData(offset+1:offset+4)),'int32'));
                        offset = offset + 4;
                        valueLength = valueLength - 4;
                        if caponCfarNumDetPoints > 0
                            caponCfarList = zeros(3, caponCfarNumDetPoints);
                            for kk = 1:caponCfarNumDetPoints
                                caponCfarList(1,kk) = double(typecast(uint8(rxData(offset+1:offset+2)),'int16')); offset = offset + 2; valueLength = valueLength - 2;
                                caponCfarList(2,kk) = double(typecast(uint8(rxData(offset+1:offset+2)),'int16')); offset = offset + 2; valueLength = valueLength - 2;
                                caponCfarList(3,kk) = double(typecast(uint8(rxData(offset+1:offset+4)),'single')); offset = offset + 4; valueLength = valueLength - 4;
                            end
                        else
                            caponCfarList = [];
                        end

                    case MMWDEMO_OUTPUT_MSG_POINT_CLOUD
                        % Point Cloud TLV - Spherical coordinates
                        % Get the unit scale for each point cloud dimension
                        pointUnit = typecast(uint8(rxData(offset+1: offset+pointUnitLengthInBytes)),'single');
                        elevUnit = pointUnit(1);
                        azimUnit = pointUnit(2);
                        dopplerUnit = pointUnit(3);
                        rangeUnit = pointUnit(4);
                        snrUnit = pointUnit(5);
                        offset = offset + pointUnitLengthInBytes;

                        numInputPoints = (valueLength - pointUnitLengthInBytes)/pointLengthInBytes;
                        if(numInputPoints > 0)
                            % Get Point Cloud from the sensor
                            pointCloudTemp = typecast(uint8(rxData(offset+1: offset+valueLength- pointUnitLengthInBytes)),'uint8');

                            rangeInfo = (double(pointCloudTemp(6:pointLengthInBytes:end)) * 256 + double(pointCloudTemp(5:pointLengthInBytes:end)));
                            rangeInfo = rangeInfo * rangeUnit;

                            azimuthInfo =  double(pointCloudTemp(2:pointLengthInBytes:end));
                            indx = find(azimuthInfo >= 128);
                            azimuthInfo(indx) = azimuthInfo(indx) - 256;
                            azimuthInfo = azimuthInfo * azimUnit; % * pi/180;

                            elevationInfo =  double(pointCloudTemp(1:pointLengthInBytes:end));
                            indx = find(elevationInfo >= 128);
                            elevationInfo(indx) = elevationInfo(indx) - 256;
                            elevationInfo = elevationInfo * elevUnit; % * pi/180;

                            dopplerInfo = double(pointCloudTemp(4:pointLengthInBytes:end)) * 256 + double(pointCloudTemp(3:pointLengthInBytes:end));
                            indx = find(dopplerInfo >= 32768);
                            dopplerInfo(indx) = dopplerInfo(indx) - 65536;
                            dopplerInfo = dopplerInfo * dopplerUnit;

                            snrInfo = double(pointCloudTemp(8:pointLengthInBytes:end)) * 256 + double(pointCloudTemp(7:pointLengthInBytes:end));
                            snrInfo = snrInfo * snrUnit;

                            idx = 1:length(dopplerInfo);  %include zero doppler

                            range = rangeInfo(idx);
                            azim = azimuthInfo(idx);
                            elev = elevationInfo(idx);
                            doppler = dopplerInfo(idx);
                            snr = snrInfo(idx);
                            pointCloudIn = [range; azim; elev; doppler; snr];

                            % Transformation from spherical to cartesian
                            point3D_T = [range.*cos(elev).*sin(azim); range.*cos(elev).*cos(azim);  range.*sin(elev)];

                            % Rotate to sensor orientation and translate to sensor position
                            point3D_W = sensorMount.Rot_TW*point3D_T;
                            point3D_W = point3D_W + sensorMount.sensorPos;
                            offset = offset + valueLength  - pointUnitLengthInBytes;
                        end                   
                    case MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS 
                        % Point Cloud TLV - Cartesian coordinates
                        % Get the unit scale for each point cloud dimension
                        pointUnit = typecast(uint8(rxData(offset+1: offset+pointUnitLengthCartesianInBytes-4)),'single');
                        xyzUnit = pointUnit(1);
                        dopplerUnit = pointUnit(2);
                        snrUnit = pointUnit(3);
                        noiseUnit = pointUnit(4);
                        numInputPts = typecast(uint8(rxData(offset+(pointUnitLengthCartesianInBytes-4)+1 : offset+pointUnitLengthCartesianInBytes)),'uint16');
                        numInputPointsMajor = numInputPts(1); %#ok
                        numInputPointsMinor = numInputPts(2); %#ok

                        offset = offset + pointUnitLengthCartesianInBytes;

                        numInputPoints = (valueLength - pointUnitLengthCartesianInBytes)/pointLengthCartesianInBytes;
                        if(numInputPoints > 0)    
                            % Get Point Cloud from the sensor
                            pointCloudTemp = typecast(uint8(rxData(offset+1: offset+valueLength - pointUnitLengthCartesianInBytes)),'uint8');
                            
                            % Received points in cartesian coordinates
                            pointCloudTemp = reshape(pointCloudTemp, pointLengthCartesianInBytes, numInputPoints);
                            
                            x = xyzUnit * double(typecast(reshape(pointCloudTemp(1:2,:), 2*numInputPoints, 1),'int16'));
                            y = xyzUnit * double(typecast(reshape(pointCloudTemp(3:4,:), 2*numInputPoints, 1),'int16'));
                            z = xyzUnit * double(typecast(reshape(pointCloudTemp(5:6,:), 2*numInputPoints, 1),'int16'));
                            doppler = dopplerUnit * double(typecast(reshape(pointCloudTemp(7:8,:), 2*numInputPoints, 1),'int16'));
                            snr = snrUnit * double(typecast(reshape(pointCloudTemp(9,:), numInputPoints, 1),'uint8'));
                            noise = noiseUnit * double(typecast(reshape(pointCloudTemp(10,:), numInputPoints, 1),'uint8')); %#ok
                            
                            range = double(sqrt(x.*x+y.*y+z.*z));
                            elev = real(double(asin(z./range))); % ignore the precision errors
                            azim = real(double(asin(x./(range.*cos(elev))))); % ignore the precision errors
                            
                            % Fill the point cloud
                            pointCloudIn = [range'; azim'; elev'; doppler'; snr'];
                            point3D_T = [double(x) double(y) double(z) ]';

                            % Rotate to sensor orientation and translate to sensor position
                            point3D_W = sensorMount.Rot_TW*point3D_T;
                            point3D_W = point3D_W + sensorMount.sensorPos;
                            offset = offset + valueLength - pointUnitLengthCartesianInBytes;
                        end

                    otherwise
                        reason = 'TLV Type is wrong or not supported';
                        offset = offset + valueLength;
                        continue;
                end
            end
            if(lostSync)
                % We came here due to wrong TLV size, restart the frame
                break;
            end
        end

        % Frame is parsed since we came to this point without any issues
        frameParsed = 1;
    end
    
    if(lostSync) && (frameParsed == 0)
        if isprop(hDataSerialPort,'BytesAvailableFcnCount')
            bytesAvailable = get(hDataSerialPort,'BytesAvailableFcnCount');
        elseif isprop(hDataSerialPort,'BytesAvailable')
            bytesAvailable = get(hDataSerialPort,'BytesAvailable');
        else
            disp('Available bytes unknown, check the serial port property');
        end
        disp(['Lost sync at frame ', num2str(targetFrameNum),'(', num2str(frameNum), '), Reason: ', reason, ', ', num2str(bytesAvailable), ' bytes in Rx buffer']);

        outOfSyncBytes = 0;
        while (lostSync)
            syncPatternFound = 1;
            for n=1:8
                [rxByte, byteCount] = readSerial(hDataSerialPort, 1, 'uint8');
                if(rxByte ~= syncPatternUINT8(n))
                    syncPatternFound = 0;
                    outOfSyncBytes = outOfSyncBytes + 1;
                    break;
                end
            end
            if(syncPatternFound == 1)
                lostSync = 0;
                [header, byteCount] = readSerial(hDataSerialPort, frameHeaderLengthInBytes - 8, 'uint8');
                rxHeader = [syncPatternUINT8, header];
                byteCount = byteCount + 8;
                gotHeader = 1;
            end
        end
    else
        break;
    end
end

if ~isempty(rngProfile), fHist(frameNum).rngProfile = rngProfile; end
if exist('antSymb', 'var') 
    if ~isempty(antSymb), fHist(frameNum).antSymb = antSymb; end 
end
if exist('detMatSlice', 'var') 
    if ~isempty(detMatSlice), fHist(frameNum).detMatSlice = detMatSlice; end 
end
if exist('snrMatSlice', 'var') 
    if ~isempty(snrMatSlice), fHist(frameNum).snrMatSlice = snrMatSlice; end 
end

if exist('sensorTimingInfo', 'var') 
    if ~isempty(sensorTimingInfo), fHist(frameNum).sensorTimingInfo = sensorTimingInfo; end 
end

if exist('caponHeatMap', 'var') 
    if ~isempty(caponHeatMap), fHist(frameNum).caponHeatMap = caponHeatMap; end 
end
if exist('caponCfarNumDetPoints', 'var') 
    if ~isempty(caponCfarNumDetPoints), fHist(frameNum).caponCfarNumDetPoints = caponCfarNumDetPoints; end 
end
if exist('caponCfarList', 'var') 
    if ~isempty(caponCfarList), fHist(frameNum).caponCfarList = caponCfarList; end 
end

end


% Utility functions
% Read the frame header
function [rxHeader, byteCount, outOfSyncBytes] = readFrameHeader(hDataSerialPort, frameHeaderLengthInBytes, syncPatternUINT8)
    lostSync = 1;
    outOfSyncBytes = 0;
    while (lostSync)
        syncPatternFound = 1;
        for n=1:8          
            [rxByte, byteCount] = readSerial(hDataSerialPort, 1, 'uint8');
            if(rxByte ~= syncPatternUINT8(n))
                syncPatternFound = 0;
                outOfSyncBytes = outOfSyncBytes + 1;
                break;
            end
        end
        if(syncPatternFound == 1)
            lostSync = 0;            
            [header, byteCount] = readSerial(hDataSerialPort, frameHeaderLengthInBytes - 8, 'uint8');
            rxHeader = [syncPatternUINT8, header];
            byteCount = byteCount + 8;
        end
    end
end

% A wrapper function to read data from serial
function [dataRead, countRead] = readSerial(device, count, datatype)
    dataRead = read(device, count, datatype);
    dataRead = cast(dataRead,datatype);
    countRead = length(dataRead);
end

% Length of a struct in bytes
function length = lengthFromStruct(S)
    fieldName = fieldnames(S);
    length = 0;
    for n = 1:numel(fieldName)
        [~, fieldLength] = S.(fieldName{n});
        length = length + fieldLength;
    end
end

% Read byte array into a struct
function [R] = readToStruct(S, ByteArray)
    fieldName = fieldnames(S);
    offset = 0;
    for n = 1:numel(fieldName)
        [fieldType, fieldLength] = S.(fieldName{n});
        R.(fieldName{n}) = typecast(uint8(ByteArray(offset+1:offset+fieldLength)), fieldType);
        offset = offset + fieldLength;
    end
end