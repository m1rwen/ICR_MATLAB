% -----------------------------------------------------------------------
% This is the Matlab visualizer for the 6843 devices to run the following in-cabin occupancy detection demos
%    (1) Intruder detection, (2) SBR, (3) CPD
%
% As this is a pre-production tool, you may experience bugs, missing/incomplete documentation and other issues.
% Please reach out to TI with feedback on any issues you encounter.
% 
% Developed by Muhammet Emin Yanik and Slobodan Jovanovic
%
% Modified: 10/10/2024 by Muhammet Emin Yanik (6843 support is added for all ID/SBR/CPD use cases)
%
% Modified: 10/23/2024 by Muhammet Emin Yanik (6844 support is added)
%
% Modified: 12/17/2024 by Muhammet Emin Yanik and Slobodan Jovanovic (6432 support is added)
% -----------------------------------------------------------------------
clear, clc, close all;
cd(fileparts(which(mfilename)))

% Set the number of devices (Hardcoded, can be done user configurable later. Support 1 or 2 only)
numDevices = 1;

% Device type: {'6843' | 'L6844' | 'L6432'};
deviceType = 'L6844';
assert(ismember(deviceType, {'6843', 'L6844', 'L6432'}), 'Error: device type is not supported!');

% Default UART speeds
controlPortBaudrate = 115200;
switch deviceType
    case '6843'
        dataPortBaudrate    = 921600;
    case 'L6844'
        dataPortBaudrate    = 1250000;
end

% GUI version
guiVersion = 'v1.33.0';

% Visualize the point cloud block data mode:
%    (0) Visualize per frame from device
%    (1) Visualize per frame from clustered in MATLAB
%    (2) Visualize across frame blocks, clustered in MATLAB
visulizePointCloud = 0;
assert(ismember(visulizePointCloud,[0,1,2]), 'Unsupported point cloud visualization mode!');

% Get the GUI folder
GUI_PATH = preparePath(visulizePointCloud);
fprintf('GUI Path: %s\n', GUI_PATH);

% The default paths required
fHistSaveFolder   = 'fHistFiles';
settingsFilePath  = 'utilityFiles\settingsCached.txt';
fhistFileName     = 'fHistRT'; % Do not include the file number (_0000, _0001, etc.) at the end

% Live Recording: {true | 'false};
liveRecording = true;

% Show video: {true | false};
showVideo = true;

% Enable ground truth car plot: 0 (No), 1 (Yes)
%  To enabled this feature, a car can be scanned and saved as a pcd file to visualize
%  Note that the coordinate system of this scan should also be aligned with the radar scene
gtruthCarPlotEnable = 0;
if (gtruthCarPlotEnable == 1)
    gTruthPath = '.\utilityFiles\example_car_scan.pcd';
    pCloud_gt = pcread(gTruthPath);
else
    pCloud_gt = [];
end

% Set the COM port settings
switch deviceType
    case '6843'
        controlPortTerminator = 'LF';
    case {'L6844', 'L6432'}
        controlPortTerminator = 'CR/LF';
end

% Use a webcam in live recording mode
if (showVideo == true) && (liveRecording == true)
    camlist = webcamlist;
    if isempty(camlist)
        disp('Warning: no camera found, video show mode is disabled in live streaming mode');
        showVideo = false;
    end
end

% Configuration Parameters
if(liveRecording == true)
    % Configuration Parameters
    settingsFileName = [GUI_PATH '\' settingsFilePath];
    fhistFilePath = [GUI_PATH '\' fHistSaveFolder];
    
    % Use single port for 6432
    if strcmp(deviceType, 'L6432') == 1
        singlePort = 1;
    else
        singlePort = 0;
    end

    % Open the config dialog 
    [controlSerialPort, dataSerialPort, chirpConfigurationFileName, loadCfg, showVideo, vidSrcId] = configDialog(settingsFileName,showVideo,GUI_PATH,singlePort);
    
    %Configure control and data UART ports
    if ~isempty(controlSerialPort) && ~isempty(dataSerialPort)
        hControlSerialPort = configureControlPort(controlSerialPort,controlPortBaudrate,controlPortTerminator);
        if controlSerialPort ~= dataSerialPort
            hDataSerialPort = configureDataPort(dataSerialPort,dataPortBaudrate);
        end
    else
        fprintf('COM ports are not selected! Exiting...\n');
        return;
    end
    if ~isfile(chirpConfigurationFileName)
        fprintf('Configuration file %s not found! Exiting...\n', chirpConfigurationFileName);
        return;
    end

    if numDevices == 2
        % Open the second device config dialog 
        [controlSerialPort2, dataSerialPort2, chirpConfigurationFileName2, loadCfg] = configDialog(settingsFileName,0,GUI_PATH,singlePort);

        %Configure control and data UART ports
        if ~isempty(controlSerialPort2) && ~isempty(dataSerialPort2)
            hControlSerialPort2 = configureControlPort(controlSerialPort2,controlPortBaudrate,controlPortTerminator);
            if controlSerialPort2 ~= dataSerialPort2
                hDataSerialPort2 = configureDataPort(dataSerialPort2,dataPortBaudrate);
            end
        else
            fprintf('COM ports are not selected! Only one device will work!\n');
            numDevices = 1;
        end
        if numDevices == 2 && ~isfile(chirpConfigurationFileName2)
            fprintf('Configuration file %s not found! Only one device will work!\n', chirpConfigurationFileName2);
            delete(hControlSerialPort2);
            delete(hDataSerialPort2);
            numDevices = 1;
        end
    end

    % Update the settings ( Use the first selected config, if there are two devices)
    chirpConfigurationFileNameToWrite = chirpConfigurationFileName;
    [configPathName,configFileName,configFileExt] = fileparts(chirpConfigurationFileNameToWrite);
    writeSettings(settingsFileName, configPathName, [configFileName configFileExt]);

    % Use a webcam in live recording mode
    if (showVideo == true)
        % Connect the webcam and configure the default resolution
        vidSource = videoinput('winvideo', vidSrcId, 'MJPG_1280x720');
    end

else
    numDevices = 1; % Run only one device
    chirpConfigurationFileName = 'test_ods_3tx_4rx.cfg';
    fhistFilePath = '';
    fhistFileName = 'fHistRT';
end

% Read Chirp Configuration file
cliCfg = readCfg(chirpConfigurationFileName);
switch deviceType
    case '6843'
        Params = parseCfg6843(cliCfg);
    case {'L6844', 'L6432'}
        Params = parseCfgL6xxx(cliCfg);
end
if numDevices == 2
    cliCfg2 = readCfg(chirpConfigurationFileName2);
    switch deviceType
        case '6843'
            Params2 = parseCfg6843(cliCfg2);
        case {'L6844', 'L6432'}
            Params2 = parseCfgL6xxx(cliCfg2);
    end
end

% Assert that the configuration parameters are the same for 2 devices, except for some options
if numDevices == 2
    ParamsCommon = rmfield(Params,'sensorPosition');
    Params2Common = rmfield(Params2,'sensorPosition');
    assert(isequal(ParamsCommon,Params2Common), 'Configuration parameters for 2 devices must be the same, except for sensor mount positions')
end

% Define the running mode and the scenes
assert(ismember(Params.runningMode,[0,1,2]), 'Unsupported running mode!');
switch Params.runningMode
    case 0
        sceneRun = 'Intrusion';
    case {1,2}
        sceneRun = 'SBR_CPD';
end

% Setup scene
switch sceneRun
    case 'Intrusion'
        scene.maxPos = [[-1.5 1.5] [-2 2.5] [0 1.5]]; %xmin,xmax, ymin,ymax, zmin,zmax
        scene.numberOfTargetBoxes = Params.occupancyBox.numBoxes;
        for n=1:scene.numberOfTargetBoxes
            box = Params.occupancyBox.box(n,:);
            scene.targetBox(n,:) = convertBoxFormat(box);
        end
        assert(scene.numberOfTargetBoxes > 1, 'Number of occupancy boxes must be at least one (inside box)');
        
        % Define a boundary box 0.5m around the car
        boundaryBoxMinLimits = [min(scene.targetBox(:,1)), min(scene.targetBox(:,2)), min(scene.targetBox(:,3))] - [0.5 0.5 0];
        boundaryBoxMaxLimits = [max(scene.targetBox(:,1)+scene.targetBox(:,4)), max(scene.targetBox(:,2)+scene.targetBox(:,5)), max(scene.targetBox(:,3)+scene.targetBox(:,6))] + [0.5 0.5 0];
        scene.boundaryBox = [boundaryBoxMinLimits boundaryBoxMaxLimits-boundaryBoxMinLimits];

    case 'SBR_CPD'        
        allCuboidX = cell2mat(arrayfun(@(a) [a.cuboid.x].', Params.zoneDef, 'UniformOutput', false));
        allCuboidY = cell2mat(arrayfun(@(a) [a.cuboid.y].', Params.zoneDef, 'UniformOutput', false));
        allCuboidZ = cell2mat(arrayfun(@(a) [a.cuboid.z].', Params.zoneDef, 'UniformOutput', false));
        boundaryBoxLimits = [min(allCuboidX(:)) max(allCuboidX(:)) min(allCuboidY(:)) max(allCuboidY(:)) min(allCuboidZ(:)) max(allCuboidZ(:))] + [-0.01 0.01 -0.01 0.01 -0.01 0.01];

        scene.boundaryBox = convertBoxFormat(boundaryBoxLimits); %x,y,z (left,near,bottom), width, depth, height
        scene.numberOfBoundaryBoxes = size(scene.boundaryBox,1);
        
        scene.numberOfTargetBoxes = Params.totNumZone;   % This is for visualization
        
        scene.totNumZone          = Params.totNumZone;           % This is for SBR related processing
        scene.zoneDef             = Params.zoneDef;

        scene.maxPos = boundaryBoxLimits + [-0.25 0.25 -0.25 0.25 0 0]; %xmin,xmax, ymin,ymax, zmin,zmax
end

% Setup the sensor orientation
% Used the same rotations: https://www.mathworks.com/help/phased/ref/rotx.html
% rotx = elev;      counterclockwise (+)
% roty = yaw;       counterclockwise (+)
% rotz = azim;      counterclockwise (+)
sensorMount.azimuthTilt    = Params.sensorPosition.azimuthTilt*pi/180;
sensorMount.elevationTilt  = Params.sensorPosition.elevationTilt*pi/180;
sensorMount.yawTilt        = Params.sensorPosition.yawTilt*pi/180;
sensorMount.sensorPos      = [Params.sensorPosition.xOffset, Params.sensorPosition.yOffset, Params.sensorPosition.zOffset]';
sensorMount.Rot_TW = getRotationMatrix(sensorMount.elevationTilt,sensorMount.yawTilt,sensorMount.azimuthTilt);
if numDevices == 2
    sensorMount2.azimuthTilt    = Params2.sensorPosition.azimuthTilt*pi/180;
    sensorMount2.elevationTilt  = Params2.sensorPosition.elevationTilt*pi/180;
    sensorMount2.yawTilt        = Params2.sensorPosition.yawTilt*pi/180;
    sensorMount2.sensorPos      = [Params2.sensorPosition.xOffset, Params2.sensorPosition.yOffset, Params2.sensorPosition.zOffset]';   
    sensorMount2.Rot_TW = getRotationMatrix(sensorMount2.elevationTilt,sensorMount2.yawTilt,sensorMount2.azimuthTilt);
end

% Sensor parameters (should be common for all sensors)
sensor.rangeMax = Params.dataPath.rangeResolutionMeters*(Params.dataPath.numRangeBins-1);
sensor.rangeMin = 0;
sensor.azimFoV = 140*pi/180; %+/- degree FOV in azimuth direction
sensor.elevFoV = 140*pi/180; %+/- degree FOV in elevation direction
sensor.framePeriod = Params.frameCfg.framePeriodicity; %in ms
sensor.rangeResolution = Params.dataPath.rangeResolutionMeters;
sensor.maxRadialVelocity = Params.dataPath.dopplerResolutionMps*Params.frameCfg.numLoops/2;
sensor.radialVelocityResolution = Params.dataPath.dopplerResolutionMps;
sensor.azim = linspace(-sensor.azimFoV/2, sensor.azimFoV/2, 128);
sensor.elev = linspace(-sensor.elevFoV/2, sensor.elevFoV/2, 128);

% Occupancy related parameters
slideWinLen = 100;

if (Params.runningMode == 0)
    if isfield(Params,'intruderDetCfg')
        occupancyThre = repmat(Params.intruderDetCfg.threshold,1,scene.numberOfTargetBoxes);
    elseif isfield(Params,'intruderDetAdvCfg')
        occupancyThre = Params.intruderDetAdvCfg.threshold;
    else
        disp('Error: intrusion detection state parameters missing, exiting!');
        return;
    end
    occBoxSignalBuffer = zeros(slideWinLen,scene.numberOfTargetBoxes);   

else
    classifierConfidenceScore = 0.5;

    if (Params.runningMode == 1)
        % Init the occupant probability buffer
        occProbSeatBuf = zeros(slideWinLen,scene.totNumZone);        
        % Init the tags buffer
        tagTemporalFiltSize = 20;
        minNumOccupancyHits = 5; % When more hits than this threshold is available, decide the occupancy
        tagTemporal = zeros(scene.totNumZone,tagTemporalFiltSize); % Initialize the tag to empty
        scene.bufferLen = 20;
    elseif (Params.runningMode == 2)
        scene.bufferLen = 30;
        % Init the adult or child probability buffers
        scoreBuf = zeros(slideWinLen,3,scene.totNumZone);
        scoreBufMA = zeros(slideWinLen,3,scene.totNumZone);
        occProbAdultBuf = zeros(slideWinLen,scene.totNumZone);
        occProbChildBuf = zeros(slideWinLen,scene.totNumZone);
        occHeightBuf    = zeros(slideWinLen,scene.totNumZone);
    end
    
    % Number of features and classes
    numFeaturesTot = 20; % Number of total features computed
    if (Params.runningMode == 1)
        numOutClasses = 2; % SBR: Empty, Occupied
        numInFeatures = 7; % Number of features used for SBR
        featuresIdx = [1,8,9,10,5,6,7]; % Indices of features to use
    elseif (Params.runningMode == 2)
        numOutClasses = 3; % CPD: Empty, Adult, Child
        numInFeatures = 17; % Number of features used for CPD
        featuresIdx = [1,4,3,2,10,9,8,13,12,11,16,15,14,19,18,17,20];
    end

    if (visulizePointCloud > 0)
        classifierBlkLen = Params.featExtrCfg.numFramesProc;
        
        featConfig.xyOffsetFlag = Params.featExtrCfg.offsetCorrection;
        featConfig.dbScanFlag = Params.featExtrCfg.dbScanFiltering;
        featConfig.dbScanEpsilon = Params.featExtrCfg.dbScanEpsilon;
        featConfig.dbScanMinPts = Params.featExtrCfg.dbScanMinPts;

        numBlkDataAvailable = 6;
        if(featConfig.xyOffsetFlag==1)
            % Keep original xy coordinates for visualization
            numBlkDataAvailable = numBlkDataAvailable + 2;
        end

        currBlkIdx = 1;
        classifierBlkData = cell(numBlkDataAvailable, scene.totNumZone, classifierBlkLen);
        if numDevices == 2
            currBlkIdx2 = 1;
            classifierBlkData2 = cell(numBlkDataAvailable, scene.totNumZone, classifierBlkLen);
        end
    end
end


% ************************************************************************
% Desktop setup
figHandle = figure('Name', 'Visualizer', 'tag', 'mainFigure');
clf(figHandle);
set(figHandle, 'WindowStyle','normal');
set(figHandle, 'WindowState','maximized');
set(figHandle,'Name',['Texas Instruments - 3D Occupancy Detection Demo Visualization ' guiVersion],'NumberTitle','off')
pause(0.1);


% Resulted tiling structure is:
%     (0)
%     |1|  |2    |  |3 |4 |   -> |Statistics | |Scene Plot    | |Heatmap + SNR  |
%     |5|  |     |  |6 |7 |      |Chirp Conf.| |              | |Range Profiles |
% (0) |8|  |9    |  |10   |      |Control    | |Occupancy     | |Ground Truth   |
figurePairs = {'Statistics',    1; ...
    'Zones Plot',               2; ...
    %'Heatmap',                  3; ...
    %'SNR',                      4; ...
    'Chirp Configuration',      5; ...
   % 'Range Profile',            6; ...
   % 'Reserved',                 7; ...
    'Control',                  8; ...
    'Ground Truth',             10};

% Add occupancy or probability signal
if (Params.runningMode == 0)
    figurePairs = [figurePairs; {'Occupancy Signal', 9}];
else
    figurePairs = [figurePairs; {'Probabilities', 9}];
end

% Add occupancy per zone plots
if (Params.runningMode == 0)
    figuresLenInit = size(figurePairs,1);
    figurePairs = [figurePairs; cell(scene.numberOfTargetBoxes-1, 2)];
    for n = 1:scene.numberOfTargetBoxes-1
        figurePairs(figuresLenInit+n,:) = {sprintf('Box%d',n), 9};
    end
    hPlotOccPerBoxTarget = zeros(scene.numberOfTargetBoxes-1,1);
end

% Add CPD Height plot
if (Params.runningMode == 2)
    figurePairs = [figurePairs; {'Height', 9}];
end

% Add debug-related plots
hMapPlotAvailable = 0;
if isfield(Params, 'dbgGuiMonitor') && (Params.dbgGuiMonitor.detMat3D == 1)
    hMapPlotAvailable = 1;
end

snrPlotAvailable = 0;
if isfield(Params, 'dbgGuiMonitor') && (Params.dbgGuiMonitor.detSnr3D == 1)
    snrPlotAvailable = 1;
end

rngPlotAvailable = 0;
if isfield(Params, 'guiMonitor') && (Params.guiMonitor.rangeProfile == 1)
    rngPlotAvailable = 1;
end


% Get the figure tiles information
figureTitles = figurePairs(:,1).';
figureTiles = cell2mat(figurePairs(:,2).');

numFigures = size(figureTitles, 2);
hFigure = zeros(1,numFigures);
hStatGlobal = [];

% Tabs: [left, bottom, width, height]
hTabGroup(1) = uitabgroup(figHandle, 'Position', [0.0 0.5 0.17 0.5]);
hTabGroup(2) = uitabgroup(figHandle, 'Position', [0.17 0.45 0.40 0.55]);
%hTabGroup(3) = uitabgroup(figHandle, 'Position', [0.57 0.6 0.215 0.4]);
%hTabGroup(4) = uitabgroup(figHandle, 'Position', [0.785 0.6 0.215 0.4]);

hTabGroup(5) = uitabgroup(figHandle, 'Position', [0.0 0.2 0.17 0.4]);
%hTabGroup(6) = uitabgroup(figHandle, 'Position', [0.57 0.4 0.215 0.2]);
%hTabGroup(7) = uitabgroup(figHandle, 'Position', [0.785 0.4 0.215 0.2]);

hTabGroup(8) = uitabgroup(figHandle, 'Position', [0.0 0.0 0.17 0.15]);
hTabGroup(9) = uitabgroup(figHandle, 'Position', [0.17 0.0 0.40 0.45]);
hTabGroup(10) = uitabgroup(figHandle, 'Position', [0.57 0.0 0.43 0.45]);

% store the figure handles
hFig = cell(numFigures,1);
exitFiguresPressed = 0;

fprintf('--------------------------Figures Are Being Prepared----------------------------\n');
for iFig = 1:numFigures
    hFigure(iFig) = uitab(hTabGroup(figureTiles(iFig)), 'Title', figureTitles{iFig});
    ax = axes('parent', hFigure(iFig));
    gca = ax;
    hFig{iFig} = ax;
        
    if(strcmp(figureTitles{iFig},'Scene Plot'))       
        axis equal;
        xlabel('X, m');
        ylabel('Y, m');
        zlabel('Z, m');
        
        % Plot the ground truth (if available)
        if ~isempty(pCloud_gt)
            pcshow(pCloud_gt,'AxesVisibility',1, 'Parent', ax, 'BackgroundColor', 'w', 'MarkerSize', 60);
        end

        axis(scene.maxPos);
        view(0,90);
        hold on;
        
        if (Params.runningMode == 0)
            for n=1:scene.numberOfTargetBoxes
                box = scene.targetBox(n,:);
                plotCube(gca, box(1:3), box(4:6), 'g','-.');
                text(gca, box(1), box(2)+0.1, sprintf('B%d',n-1), 'FontSize',9);
            end
        else
            for n=1:scene.totNumZone
                zoneDef = scene.zoneDef(n);
                for c=1:zoneDef.numCuboids
                    cuboid = zoneDef.cuboid(c).def;
                    box = convertBoxFormat(cuboid);
                    plotCube(ax, box(1:3), box(4:6), [.4 .4 .4],'-.');
                end
            end
            for n=1:scene.numberOfBoundaryBoxes
                box = scene.boundaryBox(n,:);
                plotCube(ax, box(1:3), box(4:6), 'k','-');
            end
        end

        setSensorAxesFigure(gca, sensor, sensorMount);
        if numDevices == 2
            setSensorAxesFigure(gca, sensor, sensorMount2, 'r');
        end

        grid on;
        grid minor;
        
        scenePlotAx = gca;
    end

    if(strcmp(figureTitles{iFig},'Zones Plot'))      

        % Set the number of frames to accumulate    
        if (Params.runningMode > 0)
            point3DMapAccum = cell(scene.bufferLen,scene.totNumZone);
            point3DMapAccum2 = cell(scene.bufferLen,scene.totNumZone);
        end

        axis equal;
        xlabel('X, m');
        ylabel('Y, m');
        zlabel('Z, m');
        
        % Plot the ground truth (if available)
        if ~isempty(pCloud_gt)
            pcshow(pCloud_gt,'AxesVisibility',1, 'Parent', ax, 'BackgroundColor', 'w', 'MarkerSize', 60);
        end

        axis(scene.maxPos);
        % view(6,12);
        view(0,90);
        hold on;
        
        if (Params.runningMode == 0)
            for n=1:scene.numberOfTargetBoxes
                box = scene.targetBox(n,:);
                plotCube(gca, box(1:3), box(4:6), 'g','-.');
                text(gca, box(1), box(2)+0.1, sprintf('B%d',n-1), 'FontSize',9);
            end
        else
            for n=1:scene.totNumZone
                zoneDef = scene.zoneDef(n);
                for c=1:zoneDef.numCuboids
                    cuboid = zoneDef.cuboid(c).def;
                    box = convertBoxFormat(cuboid);
                    plotCube(ax, box(1:3), box(4:6), [.4 .4 .4],'-.');
                end
            end
            for n=1:scene.numberOfBoundaryBoxes
                box = scene.boundaryBox(n,:);
                plotCube(ax, box(1:3), box(4:6), 'k','-');
            end
        end

        setSensorAxesFigure(gca, sensor, sensorMount);
        if numDevices == 2
            setSensorAxesFigure(gca, sensor, sensorMount2, 'r');
        end

        grid on;
        grid minor;
        
        zonePlotAx = gca;
    end
    
    if(strcmp(figureTitles{iFig},'Heatmap'))
        if hMapPlotAvailable == 0
            configureDisabledWindow(gca)
        else
            heatmap3dAx = gca;
            axis equal;
            xlabel('X, m');
            ylabel('Y, m');
            zlabel('Z, m');
            axis(scene.maxPos);
            hold on;
            
            for n=1:scene.numberOfTargetBoxes
                box = scene.targetBox(n,:);
                plotCube(gca, box(1:3), box(4:6), 'r','-.');
            end
        end
    end
    
    if(strcmp(figureTitles{iFig},'SNR'))
        if snrPlotAvailable == 0
            configureDisabledWindow(gca)
        else
            snr3dAx = gca;
            axis equal;
            xlabel('X, m');
            ylabel('Y, m');
            zlabel('Z, m');
            axis(scene.maxPos);
            hold on;

            for n=1:scene.numberOfTargetBoxes
                box = scene.targetBox(n,:);
                plotCube(gca, box(1:3), box(4:6), 'r','-.');
            end
        end
    end

    if(strcmp(figureTitles{iFig},'Reserved'))
         configureDisabledWindow(gca,'Reserved for future use!');
    end
        
    if(strcmp(figureTitles{iFig},'Range Profile'))
        if rngPlotAvailable == 0
            configureDisabledWindow(gca)
        else
            hPlotRangeProfile = plot(Params.dataPath.rangeResolutionMeters*(0:Params.dataPath.numRangeBins-1),zeros(Params.dataPath.numRangeBins,1));
            xlabel('Range, m');
            ylabel('dB');
            set(gca, 'XLimSpec', 'Tight');
            a=axis;
            a(3)=20;    %dB
            a(4)=80;    %dB
            axis(a);
            grid on
        end
    end
    
    if (strcmp(figureTitles{iFig},'Occupancy Signal'))
        hold on

        hPlotOccBoxTarget = zeros(scene.numberOfTargetBoxes-1,1);
        for n=1:scene.numberOfTargetBoxes-1
            hPlotOccBoxTarget(n) = plot(gca, 1:slideWinLen, zeros(1,slideWinLen), 'linewidth', 1);
        end
        yline(gca, occupancyThre, 'b-.', 'linewidth', 0.5)

        legend([sprintfc('TargetBox %d',1:scene.numberOfTargetBoxes-1),'Threshold'],'Location','northwest')
        xlabel(hFig{iFig},'Frame number');
        ylabel(hFig{iFig},'Occupancy Signal');

        set(gca, 'XLimSpec', 'Tight');
        grid on
    end
    
    if (startsWith(figureTitles{iFig},'Box'))
        boxId = str2double(strtok(figureTitles{iFig},'Box'));
        hold on

        hPlotOccPerBoxTarget(boxId) = plot(gca, 1:slideWinLen, zeros(1,slideWinLen), 'linewidth', 1);
        yline(gca, occupancyThre(boxId+1), 'b-.', 'linewidth', 0.5)

        legend({'Signal','Threshold'},'Location','northwest')
        xlabel(hFig{iFig},'Frame number');
        ylabel(hFig{iFig},'Occupancy Signal');

        set(gca, 'XLimSpec', 'Tight');
        grid on
    end

    if (strcmp(figureTitles{iFig},'Probabilities'))
        hold on

        hPlotOccProbSeat = zeros(scene.totNumZone,1);
        markerList = {'o', '+', 'x', 'square', 'diamond', '^', 'v', 'pentagram'};

        for n=1:scene.totNumZone
            hPlotOccProbSeat(n) = plot(gca, 1:slideWinLen, zeros(1,slideWinLen), ['-' markerList{n}], 'linewidth', 1, 'MarkerSize', 3);
        end
        yline(gca, classifierConfidenceScore, 'b-.', 'linewidth', 0.5)

        legend([sprintfc('Seat %d',1:scene.totNumZone),'Threshold'],'Location','northwest')
        xlabel(hFig{iFig},'Frame number');
        ylabel(hFig{iFig},'Occupancy Probability (Per Classifier Run)');
        ylim([-0.1 1.1]);

        % Set control panel for adult/child
        if (Params.runningMode == 2)
            hCbChildEnabled = uicontrol(hFigure(iFig),'Style', 'checkbox', 'String', 'Child (or Adult)','FontSize', 10,...
                'Units', 'normalized','Position', [0.02 0.95 0.2 0.04],'Value',1);
        end

        set(gca, 'XLimSpec', 'Tight');
        grid on
    end

    if (strcmp(figureTitles{iFig},'Height'))
        heightAx = gca;
        hold on
        
        hPlotHeightSeat = zeros(scene.totNumZone,1);
        markerList = {'o', '+', 'x', 'square', 'diamond', '^', 'v', 'pentagram'};
       
        for n=1:scene.totNumZone
            hPlotHeightSeat(n) = plot(heightAx, 1:slideWinLen, zeros(1,slideWinLen), ['-' markerList{n}], 'linewidth', 1, 'MarkerSize', 3);
        end

        legend(sprintfc('Seat %d',1:scene.totNumZone),'Location','northwest')
        xlabel(hFig{iFig},'Frame number');
        ylabel(hFig{iFig},'Height (cm)');

        set(gca, 'XLimSpec', 'Tight');
        grid on
    end

    if(strcmp(figureTitles{iFig},'Chirp Configuration'))
        set(gca, 'visible', 'off');
        tablePosition = [0.05 0.05 0.86 0.86];
        h = displayChirpParams(Params, tablePosition, hFigure(iFig));
    end
    
    if(strcmp(figureTitles{iFig},'Statistics'))     
        set(gca, 'visible', 'off');
        fontSize = 10;

        hStatGlobal(1) = text(0, 0.95, 'Frame: 0 (0)', 'FontSize',fontSize);
        hStatGlobal(2) = text(0, 0.9, 'Detection Points: 0 (0)','FontSize',fontSize);        
        hStatGlobal(3) = text(0, 0.8, 'Occupancy: 0','FontSize',fontSize);
        hStatGlobal(4) = text(0, 0.7, 'Rx Buffer, bytes: 0','FontSize',fontSize);
        hStatGlobal(5) = text(0, 0.65, 'ARM, ms: 0.0','FontSize',fontSize);
        hStatGlobal(6) = text(0, 0.6, 'DSP, ms: 0.0','FontSize',fontSize);
        hStatGlobal(7) = text(0, 0.55, 'UART, ms: 0.0','FontSize',fontSize);
        hStatGlobal(8) = text(0, 0.5, 'Processing Load, %: 0.0','FontSize',fontSize);
        hStatGlobal(9) = text(0, 0.45, 'GUI, ms: 0.0','FontSize',fontSize);
    end
    
    if(strcmp(figureTitles{iFig},'Control'))
        set(gca, 'visible', 'off');

        hRbPause = uicontrol(hFigure(iFig),'Style','radio','String','Pause','FontSize', 10,...
            'Units', 'normalized', 'Position',[0.1 0.3 0.3 0.2],'Value',0);
        hPbExit = uicontrol(hFigure(iFig),'Style', 'pushbutton', 'String', 'Exit','FontSize', 10,...
            'Units', 'normalized','Position', [0.6 0.2 0.3 0.3],'Callback', @exitPressFcn);
        setappdata(hPbExit, 'exitKeyPressed', 0);
    end

    if (strcmp(figureTitles{iFig},'Ground Truth'))
        if showVideo == false
            configureDisabledWindow(gca,'Video is disabled!');
        else
            set(gca, 'visible', 'off');
            set(gca, 'Position', [0 0 1 1]);
            groundTruthAx = gca;
        end
    end
end
fprintf('-------------------------------Figures Created----------------------------------\n');
pause(1);

% Display the video preview.
if (showVideo == true) && (liveRecording == true)
    vidRes = vidSource.VideoResolution;
    nBands = vidSource.NumberOfBands;
    hImage = imshow(zeros(vidRes(2), vidRes(1), nBands, 'uint8'),'InitialMagnification', 'fit','Border','tight','Parent',groundTruthAx);
    
    % Display the video data in your GUI.
    preview(vidSource, hImage);
end

fileLoop = 0;
fileFrameSize = 1000;

% Frame history structure
frameStatStruct = struct('targetFrameNum', [], 'bytes', [], 'numInputPoints', 0, 'numOutputPoints', 0, 'timestamp', 0, 'start', 0, 'benchmarks', zeros(10,1), 'pointCloud', []);
fHist = repmat(frameStatStruct, 1, fileFrameSize);
if numDevices == 2
    fHist2 = repmat(frameStatStruct, 1, fileFrameSize);
end

if(liveRecording == true)
    %Send Configuration Parameters to the device
    if loadCfg == 1
        % Send CLI configuration
        fprintf('Sending configuration from %s file to Target EVM 1 ...\n', chirpConfigurationFileName);
        [hControlSerialPort, sensorStartCmd] = sendCfgToDevice(hControlSerialPort, cliCfg);

        if numDevices == 2
            fprintf('Sending configuration from %s file to Target EVM 2 ...\n', chirpConfigurationFileName2);
            [hControlSerialPort2, sensorStartCmd2] = sendCfgToDevice(hControlSerialPort2, cliCfg);
        end
    else
        for k=1:length(cliCfg)
            if strcmp('baudRate',strtok(cliCfg{k},' '))
                [~, baudRate] = strtok(cliCfg{k},' ');
                set(hControlSerialPort, 'BaudRate', str2double(baudRate));
                if numDevices == 2
                    set(hControlSerialPort2, 'BaudRate', str2double(baudRate));
                end
                break;
            end
        end
    end

    % Logging folder
    if ~exist(fhistFilePath,'dir')
        if mkdir(fhistFilePath)
        else
            error(strcat('cannot created folder', fhistFilePath, '!'));
            return;
        end
    end
end


% Figure handles
if (Params.runningMode == 0)
    hTargetBoxHandle = zeros(scene.numberOfTargetBoxes,1);
    hPlotCloudHandleAll = [];
else
    hTargetBoxHandle = zeros(scene.totNumZone,max([scene.zoneDef.numCuboids]));
    hPlotCloudHandleAll = [];
    hPlotCloudHandleZones = cell(scene.totNumZone,1);
end
if numDevices == 2
    hPlotCloudHandleAll2 = [];
    hPlotCloudHandleZones2 = cell(scene.totNumZone,1);
end
hPlotHeatmap3d = [];
hPlotSnr3d = [];

frameNum = 1;
targetFrameNum = 1;
numFramesTotal = 0;

pointCloud = single(zeros(5,0));
point3Dw = single(zeros(3,0));
if numDevices == 2
    targetFrameNum2 = 1;
    pointCloud2 = single(zeros(5,0));
    point3Dw2 = single(zeros(3,0));
end

if(liveRecording == false)
    matlabFileName = [fhistFilePath, '\', fhistFileName, '_', num2str(fileLoop, '%04d'), '.mat'];
    if(isfile(matlabFileName))
        load(matlabFileName,'fHist');
        disp(['Loading data from ', matlabFileName, ' ...']);
        fileFrameSize = size(fHist,2);
    else
        disp('Exiting');
        return;
    end
end

% Everything is ready, start the sensor
if (liveRecording == true) && loadCfg == 1
    msgfig = msgbox('Device and GUI are both configured successfully! Click OK to start!','Sensor Start');
    uiwait(msgfig)
    writeLineSerial(hControlSerialPort, sensorStartCmd);
    if numDevices == 2
        writeLineSerial(hControlSerialPort2, sensorStartCmd2);
    end
end

% Update the data port for single port devices
if (liveRecording == true)
    if controlSerialPort == dataSerialPort
        hDataSerialPort = reconfigureControlPort(hControlSerialPort);
    end
    if numDevices == 2
        if controlSerialPort2 == dataSerialPort2
            hDataSerialPort2 = reconfigureControlPort(hControlSerialPort2);
        end
    end
end

% Increase the timeout if in the ADC data debug mode
if Params.adcDataSource.isSourceFromFile
    set(hDataSerialPort, 'Timeout', 600);
end

fprintf('-------------------------------Frames Started----------------------------------\n');

frameStart = tic;
while(1)
    if(liveRecording == true)
        bmCount = 1;

        % Get and parse the TLVs from the device (set 10 frames timeout)
        [fHist, targetFrameNum, pointCloudIn, point3DwIn, intrusionDetInfo, classifierInfo] = parseDataFromDevice(hDataSerialPort, fHist, frameNum, targetFrameNum, sensorMount, frameStart);
        fHist(frameNum).benchmarks(bmCount) = 1000*toc(frameStart);

        if numDevices == 2
            [fHist2, targetFrameNum2, pointCloudIn2, point3DwIn2, intrusionDetInfo2, classifierInfo2] = parseDataFromDevice(hDataSerialPort2, fHist2, frameNum, targetFrameNum2, sensorMount2, frameStart);
            fHist2(frameNum).benchmarks(bmCount) = 1000*toc(frameStart);
        end
        bmCount = bmCount + 1;
        
        % Get the point cloud
        pointCloud = pointCloudIn;
        point3Dw = point3DwIn;
        numInputPoints = size(pointCloud,2);
        numOutputPoints = size(pointCloud,2);
        if numDevices == 2
            pointCloud2 = pointCloudIn2;
            point3Dw2 = point3DwIn2;
            numInputPoints2 = size(pointCloud2,2);
            numOutputPoints2 = size(pointCloud2,2);
        end

        % Store Point cloud
        fHist(frameNum).numInputPoints = numInputPoints;
        fHist(frameNum).numOutputPoints = numOutputPoints;
        fHist(frameNum).pointCloud = pointCloud;
        fHist(frameNum).point3Dw = point3Dw;
        if numDevices == 2
            fHist2(frameNum).numInputPoints = numInputPoints2;
            fHist2(frameNum).numOutputPoints = numOutputPoints2;
            fHist2(frameNum).pointCloud = pointCloud2;
            fHist2(frameNum).point3Dw = point3Dw2;
        end
    else
        % Only for one device
        targetFrameNum = fHist(frameNum).targetFrameNum;
        numInputPoints = fHist(frameNum).numInputPoints;
        numOutputPoints = fHist(frameNum).numOutputPoints;
        pointCloud = fHist(frameNum).pointCloud;
        point3Dw = fHist(frameNum).point3Dw;
    end

    % Check pause status
    if isvalid(hRbPause) && (get(hRbPause, 'Value') == 1)
        pause(0.1);
        continue;
    end

    % Check the figure handles, exit if any figure is closed
    if all(ishandle([hFig{:}]))
        % Update the point cloud for visualization
        if (visulizePointCloud == 0) || (Params.runningMode == 0)
            point3Dw_plot = point3Dw;
            if (numDevices == 2)
                point3Dw2_plot = point3Dw2;
            end
        else
            [point3Dw_plot, classifierBlkData, currBlkIdx] = pointCloudVisualization(point3Dw, pointCloud, scene, visulizePointCloud, featConfig, classifierBlkData, currBlkIdx);
            if (numDevices == 2)
                [point3Dw2_plot, classifierBlkData2, currBlkIdx2] = pointCloudVisualization(point3Dw2, pointCloud2, scene, visulizePointCloud, featConfig, classifierBlkData2, currBlkIdx2);
            end
        end

%         % Plot points in point cloud scene window
%         hPlotCloudHandleAll = plotPointCloud(scenePlotAx, point3Dw_plot, hPlotCloudHandleAll, 'r', 40);
%         if (numDevices == 2)
%             hPlotCloudHandleAll2 = plotPointCloud(scenePlotAx, point3Dw2_plot, hPlotCloudHandleAll2, 'b', 40);
%         end

        % Plot points in point cloud zone window
        % Assign (map) detected points to the defined zone cuboids
        if ( Params.runningMode == 0)
            hPlotCloudHandleAll = plotPointCloud(zonePlotAx, point3Dw_plot, hPlotCloudHandleAll, 'r', 40);
            if (numDevices == 2)
                hPlotCloudHandleAll2 = plotPointCloud(zonePlotAx, point3Dw2_plot, hPlotCloudHandleAll2, 'b', 40);
            end

        else
            [zoneMap, ~, ~, ~] = zone_assign(point3Dw_plot, scene.totNumZone, scene.zoneDef);
            if (numDevices == 2)
                [zoneMap2, ~, ~, ~] = zone_assign(point3Dw2_plot, scene.totNumZone, scene.zoneDef);
            end
    
            % Accumulate points across a given number of frames
            frameModIdx = mod(frameNum, scene.bufferLen) + 1;
            for i = 1:scene.totNumZone
                point3DMapAccum{frameModIdx,i} = point3Dw_plot(:,zoneMap(:,i) == 1); % Buffer Len x num zones
                if (numDevices == 2)
                    point3DMapAccum2{frameModIdx,i} = point3Dw2_plot(:,zoneMap2(:,i) == 1); % Buffer Len x num zones
                end
            end        

            hPlotCloudHandleZones = plotPointCloudZones(zonePlotAx, point3DMapAccum, hPlotCloudHandleZones);
            if (numDevices == 2)
                hPlotCloudHandleZones2 = plotPointCloud(zonePlotAx, point3DMapAccum2, hPlotCloudHandleZones2);
            end
        end 

        if(liveRecording == true)
            fHist(frameNum).benchmarks(bmCount) = 1000*toc(frameStart);
            if (numDevices == 2)
                fHist2(frameNum).benchmarks(bmCount) = 1000*toc(frameStart);
            end
            bmCount = bmCount + 1;
        end

        if (Params.runningMode == 0)
            % Update the occupancy plots
            assert(intrusionDetInfo.numOccBoxes == scene.numberOfTargetBoxes, 'Number of boxes in the received signal does not match with the configuration!');

            % Inside is the first occupancy box, the rest is the target boxes
            occBoxSignalBuffer        = circshift(occBoxSignalBuffer,-1,1);
            occBoxSignalBuffer(end,:) = intrusionDetInfo.occBoxSignal;

            for n=1:scene.numberOfTargetBoxes-1
                set(hPlotOccBoxTarget(n), 'Ydata', occBoxSignalBuffer(:,n+1));
                set(hPlotOccPerBoxTarget(n), 'Ydata', occBoxSignalBuffer(:,n+1));
            end

            % Overall occupancy result
            occupancyStatus = any(intrusionDetInfo.occBoxDecision==1);

            % Plot occupancy state based on the threshold
            for n=1:scene.numberOfTargetBoxes
                if( (hTargetBoxHandle(n) ~= 0) && (ishandle(hTargetBoxHandle(n))))
                    delete(hTargetBoxHandle(n));
                end
                if (occupancyStatus == 1)
                    box = scene.targetBox(n,:);
                    if (intrusionDetInfo.occBoxDecision(n) == 1)
                        %hTargetBoxHandle(n) = plotCube(scenePlotAx, box(1:3), box(4:6), 'r', '-', 2, 'r', 0.1);
                        hTargetBoxHandle(n) = plotCube(zonePlotAx, box(1:3), box(4:6), 'r', '-', 2, 'r', 0.1);
                    else
                        %hTargetBoxHandle(n) = plotCube(scenePlotAx, box(1:3), box(4:6), 'r', '-', 2, 'r', 0.02);
                        hTargetBoxHandle(n) = plotCube(zonePlotAx, box(1:3), box(4:6), 'r', '-', 2, 'r', 0.02);
                    end
                end
            end

            % Save the result in the fHist
            fHist(frameNum).occBoxSignal = intrusionDetInfo.occBoxSignal;
            fHist(frameNum).occBoxDecision = intrusionDetInfo.occBoxDecision;
            fHist(frameNum).occupancyStatus = occupancyStatus;

            % Save and plot the heatmap if enabled
            if isfield(intrusionDetInfo,'detMat3D')
                dataReshaped = reshape(intrusionDetInfo.detMat3D, Params.dataPath.azimuthFftSize, Params.dataPath.numRangeBins, Params.dataPath.elevationFftSize);
                dataReshaped = permute(dataReshaped,[2,1,3]);
                fHist(frameNum).detMat3D = dataReshaped;

                sceneOccupancy.Rot_TW = sensorMount.Rot_TW;
                sceneOccupancy.sensorPos = sensorMount.sensorPos.';
                sceneOccupancy.boundaryBox = scene.boundaryBox(1,:);
                [heatmapCartesian, xDim,yDim,~] = assignHeatmapToBoundary(dataReshaped, Params.dataPath.rangeVal, Params.dataPath.azimVal, Params.dataPath.elevVal, sceneOccupancy);

                % Plot the heatmap
                heatmapCartesian(isnan(heatmapCartesian)) = 0;
                if isempty(hPlotHeatmap3d) || ~ishandle(hPlotHeatmap3d)
                    hPlotHeatmap3d = mesh(heatmap3dAx, xDim,yDim,zeros(length(yDim),length(xDim)), 'FaceColor','interp','LineStyle','none');
                    colormap jet
                end
                set(hPlotHeatmap3d, 'CData', max(heatmapCartesian,[],3));
            end
            
            % Save and plot the SNR if enabled
            if isfield(intrusionDetInfo,'detSnr3D')
                dataReshaped = reshape(intrusionDetInfo.detSnr3D, Params.dataPath.numRangeBins, Params.dataPath.azimuthFftSize, Params.dataPath.elevationFftSize);
                fHist(frameNum).detSnr3D = dataReshaped;
                
                sceneOccupancy.Rot_TW = sensorMount.Rot_TW;
                sceneOccupancy.sensorPos = sensorMount.sensorPos.';
                sceneOccupancy.boundaryBox = scene.boundaryBox(1,:);
                [heatmapCartesian, xDim,yDim,~] = assignHeatmapToBoundary(dataReshaped, Params.dataPath.rangeVal, Params.dataPath.azimVal, Params.dataPath.elevVal, sceneOccupancy);

                % Plot the heatmap
                heatmapCartesian(isnan(heatmapCartesian)) = 0;
                if isempty(hPlotSnr3d) || ~ishandle(hPlotSnr3d)
                    hPlotSnr3d = mesh(snr3dAx, xDim,yDim,zeros(length(yDim),length(xDim)), 'FaceColor','interp','LineStyle','none');
                    colormap jet
                end
                set(hPlotSnr3d, 'CData', max(heatmapCartesian,[],3));
            end
            
        else
            % SBR or CPD mode
            tagCurrent = zeros(scene.totNumZone,1);

            if ~isempty(classifierInfo)
                % Save the features in the fHist
                occFeaturesShape = reshape(classifierInfo.occFeatures, numFeaturesTot, []);
                fHist(frameNum).occupancy.featureSet = occFeaturesShape(featuresIdx,:);

                % Default prediction and decision status of current frame
                occPredictions = reshape(classifierInfo.occPredictions, numOutClasses, []);

                if (Params.runningMode == 1)
                    % Classify all the seats through the temporal filter
                    scoreOcc = zeros(scene.totNumZone,1);
                    for z = 1:scene.totNumZone
                        score = occPredictions(:,z);
                        [scoreOcc(z), tagCurrent(z), tagTemporal(z,:)] = classifierTemporalFilter(score, classifierConfidenceScore, minNumOccupancyHits, tagTemporal(z,:));
                    end

                    % Update the plot of the decision probabilities
                    occProbSeatBuf        = circshift(occProbSeatBuf,-1,1);
                    occProbSeatBuf(end,:) = scoreOcc;

                    for n=1:scene.totNumZone
                        set(hPlotOccProbSeat(n), 'Ydata', occProbSeatBuf(:,n));
                    end

                elseif (Params.runningMode == 2)
                    % Classify all the seats based on the existing predictions
                    scoreOcc = zeros(scene.totNumZone,2);
                    % scoreOccMA = zeros(scene.totNumZone,2);
                    for z = 1:scene.totNumZone
                        score = occPredictions(:,z);
                        if score(1)>classifierConfidenceScore || all(score==0)
                            tagCurrent(z) = 0;
                        elseif score(2)>classifierConfidenceScore
                            tagCurrent(z) = 1;
                        else
                            tagCurrent(z) = 2;
                        end
                        scoreOcc(z,:) = score(2:3);

                        % Use moving average for scores
                        % scoreBuf = circshift(scoreBuf,-1,1);
                        % scoreBuf(end,:,z) = score;
                        % scoreBufMA(:,:,z) = movmean(scoreBuf(:,:,z),10,1); % Moving average for scores
                        % scoreMA = scoreBufMA(end,:,z);
                        % [~,scoreMax] = max(scoreMA);
                        % 
                        % if scoreMax == 1
                        %     tagCurrent(z) = 0;
                        % elseif scoreMax == 2
                        %     tagCurrent(z) = 1;
                        % else
                        %     tagCurrent(z) = 2;
                        % end
                        % 
                        % scoreOccMA(z,:) = scoreMA(2:3);
                    end

                    % Update the plot of the decision probabilities
                    occProbAdultBuf        = circshift(occProbAdultBuf,-1,1);
                    occProbAdultBuf(end,:) = scoreOcc(:,1);
                    % occProbAdultBuf(end,:) = scoreOccMA(:,1);
                    occProbChildBuf        = circshift(occProbChildBuf,-1,1);
                    occProbChildBuf(end,:) = scoreOcc(:,2);
                    % occProbChildBuf(end,:) = scoreOccMA(:,2);

                    if (get(hCbChildEnabled, 'Value') == 1)
                        for n=1:scene.totNumZone
                            set(hPlotOccProbSeat(n), 'Ydata', occProbChildBuf(:,n));
                        end
                    else
                        for n=1:scene.totNumZone
                            set(hPlotOccProbSeat(n), 'Ydata', occProbAdultBuf(:,n));
                        end
                    end

                    % Height
                    occHeightBuf        = circshift(occHeightBuf,-1,1);
                    occHeightBuf(end,:) = classifierInfo.occHeight;

                    for n=1:scene.totNumZone
                        set(hPlotHeightSeat(n), 'Ydata', occHeightBuf(:,n));
                    end
                end

                % Save the classifier result in the fHist
                fHist(frameNum).occupancy.scoresOcc = scoreOcc;
                fHist(frameNum).occupancy.tags      = tagCurrent;
                if (Params.runningMode == 2)
                    fHist(frameNum).occupancy.height = classifierInfo.occHeight;
                end

                for z=1:scene.totNumZone
                    zoneDef = scene.zoneDef(z);
                    for c=1:zoneDef.numCuboids
                        if( (hTargetBoxHandle(z,c) ~= 0) && (ishandle(hTargetBoxHandle(z,c))))
                            delete(hTargetBoxHandle(z,c));
                        end

                        if(tagCurrent(z) == 1)
                            % Occupied for SBR, Adult for CPD
                            cuboid = zoneDef.cuboid(c).def;
                            box = convertBoxFormat(cuboid);
                            %hTargetBoxHandle(z,c) = plotCube(scenePlotAx, box(1:3), box(4:6), 'g','-',2, 'g',0.3);
                            hTargetBoxHandle(z,c) = plotCube(zonePlotAx, box(1:3), box(4:6), 'g','-',2, 'g',0.3);

                        elseif(tagCurrent(z) == 2)
                            % Child for CPD
                            cuboid = zoneDef.cuboid(c).def;
                            box = convertBoxFormat(cuboid);
                            %hTargetBoxHandle(z,c) = plotCube(scenePlotAx, box(1:3), box(4:6), 'b','-',2, 'b',0.3);
                            hTargetBoxHandle(z,c) = plotCube(zonePlotAx, box(1:3), box(4:6), 'b','-',2, 'b',0.3);
                        end
                    end
                end

            end
        end

        % Plot Range Profile
        if isfield(fHist(frameNum),'rngProfile') && ~isempty(hPlotRangeProfile)
            % Convert to dB
            set(hPlotRangeProfile, 'Ydata', 20*log10(fHist(frameNum).rngProfile));
        end

        % For debugging: plots angle heatmap based on received 16 antenna symbols (corresponding 
        % to corner reflector at 1 meter from the sensor, and assuming range step of 0.048 meters/bin)
        if isfield(fHist(frameNum),'antSymb')
            if ~exist('hAngleHeatmapDbg', 'var')
                figure
                antGeomTable = [4 5 8 9 7 6 11 10 0 1 12 13 3 2 15 14]+1;
                antGeomSignTable = [1 1 1 1 -1 -1 -1 -1 1 1 1 1 -1 -1 -1 -1 ];
                hAngleHeatmapDbg = surf(-16:15 , -16:15, zeros(32));
                view(2);
                axis tight;
            end
            antSymHM = fHist(frameNum).antSymb .* antGeomSignTable;
            antSymHM = reshape(antSymHM(antGeomTable), 4, 4);
            antSymHM = fftshift(abs(fft(fft(antSymHM,32,1), 32,2)));
            set(hAngleHeatmapDbg, 'zData', antSymHM);
        end

        % For debugging: Plots the angle heatmap slice from the 3D detection matrix
        % at range 1 meter (assuming range step of 0.048 meters/bin)
        if isfield(fHist(frameNum),'detMatSlice')
            if ~exist('hdetMatSliceDbg', 'var')
                figure
                hdetMatSliceDbg = surf(-8:7 , -8:7, zeros(16));
                view(2);
                axis tight;
                title('Detection Matrix slice')
            end
            set(hdetMatSliceDbg, 'zData', fHist(frameNum).detMatSlice);
        end

        % For debugging: Plots the angle heatmap slice from the 3D SNR matrix
        % at range 1 meter (assuming range step of 0.048 meters/bin)
        if isfield(fHist(frameNum),'snrMatSlice')
            if ~exist('hsnrMatSliceDbg', 'var')
                figure
                hsnrMatSliceDbg = surf(-8:7 , -8:7, zeros(16));
                view(2);
                axis tight;
                title('SNR slice')
            end
            set(hsnrMatSliceDbg, 'zData', fHist(frameNum).snrMatSlice);
        end


        if(liveRecording == true)
            fHist(frameNum).benchmarks(bmCount) = 1000*toc(frameStart);
            if (numDevices == 2)
                fHist2(frameNum).benchmarks(bmCount) = 1000*toc(frameStart);
            end
            bmCount = bmCount + 1;
        end
        
        % TODO
        string{1} = sprintf('Frame: %d (%d)', frameNum, targetFrameNum);
        string{2} = sprintf('Detection Points: %d (%d)', numInputPoints, numOutputPoints);

        if (Params.runningMode == 0)
            string{3} = sprintf('Occupancy: %d', occupancyStatus);
        else
            string{3} = sprintf(['Occupancy: %d',repmat('-%d',1,scene.totNumZone-1)], tagCurrent);
        end

        string{4} = sprintf('Rx Buffer, bytes: %d', 0);
        if isfield(fHist, 'sensorTimingInfo')
            string{5} = sprintf('ARM, ms: %.1f', (fHist(frameNum).sensorTimingInfo.interFrameProcessingTime_usec - ...
                                                  fHist(frameNum).sensorTimingInfo.dspProcessingTime_usec)/1000);
            string{6} = sprintf('DSP, ms: %.1f', fHist(frameNum).sensorTimingInfo.dspProcessingTime_usec/1000);
            string{7} = sprintf('UART, ms: %.1f', fHist(frameNum).sensorTimingInfo.transmitOutputTime_usec/1000);
        else
            string{5} = sprintf('ARM, ms: %.1f', 0);
            string{6} = sprintf('DSP, ms: %.1f', 0);
            string{7} = sprintf('UART, ms: %.1f', 0);
        end
        string{8} = sprintf('Processing Load, %%: %.1f', 0);
        string{9} = sprintf('GUI, ms: %.1f', 0);         

        for n=1:length(hStatGlobal)
            set(hStatGlobal(n),'String',string{n});
        end

    else
        exitFiguresPressed = 1;
    end

    if (isvalid(hPbExit) && (getappdata(hPbExit, 'exitKeyPressed') == 1)) || exitFiguresPressed
        if(liveRecording == true)
            if (showVideo == true)
                stop(vidSource);
                delete(vidSource)
                clear vidSource;
            end

            matlabFileName = [fhistFilePath, '\', fhistFileName, '_', num2str(fileLoop, '%04d'), '.mat'];
            fHist = fHist(1:frameNum);
            save(matlabFileName,'fHist','scene','sensorMount');
            disp(['Saving data in ', matlabFileName, ' ...']);
            % Combine all the fHist files
            if(fileLoop > 0)
                combineFhistFiles(fileparts(matlabFileName), fhistFileName);
            end
            if numDevices == 2
                matlabFileName = [fhistFilePath, '\', fhistFileName '2_', num2str(fileLoop, '%04d'), '.mat'];
                fHist2 = fHist2(1:frameNum);
                save(matlabFileName,'fHist2','scene','sensorMount');
                disp(['Saving data in ', matlabFileName, ' ...']);
                % Combine all the fHist files
                if(fileLoop > 0)
                    combineFhistFiles(fileparts(matlabFileName), [fhistFileName '2']);
                end
            end
        end
        disp('Exiting');
        close all;
        return;
    end

    frameNum = frameNum + 1;

    if(frameNum > fileFrameSize)
        if(liveRecording == true)
            matlabFileName = [fhistFilePath, '\', fhistFileName, '_', num2str(fileLoop, '%04d'), '.mat'];
            save(matlabFileName,'fHist','scene','sensorMount');
            disp(['Saving data in ', matlabFileName, ' ...']);
            fHist = repmat(frameStatStruct, 1, fileFrameSize);
            numFramesTotal = numFramesTotal + fileFrameSize;
            if numDevices == 2
                matlabFileName = [fhistFilePath, '\', fhistFileName '2_', num2str(fileLoop, '%04d'), '.mat'];
                save(matlabFileName,'fHist2','scene','sensorMount');
                disp(['Saving data in ', matlabFileName, ' ...']);
                fHist2 = repmat(frameStatStruct, 1, fileFrameSize);
            end
        else
            matlabFileName = [fhistFilePath, '\', fhistFileName, '_', num2str(fileLoop + 1, '%04d'), '.mat'];
            if(isfile(matlabFileName))
                load(matlabFileName,'fHist');
                disp(['Loading data from ', matlabFileName, ' ...']);
                numFramesTotal = numFramesTotal + fileFrameSize;
                fileFrameSize = size(fHist,2);
            else
                disp('Exiting');
                return;
            end
        end
        frameNum = 1;
        fileLoop = fileLoop + 1;
    end

    if  Params.adcDataSource.isSourceFromFile && ((numFramesTotal+frameNum) > Params.adcDataSource.numFrames) && (liveRecording == true)
        matlabFileName = [fhistFilePath, '\', fhistFileName, '_', num2str(fileLoop, '%04d'), '.mat'];
        fHist = fHist(1:frameNum-1);
        save(matlabFileName,'fHist');
        disp(['Saving data in ', matlabFileName, ' ...']);

        % Combine all the fHist files
        if(fileLoop > 0)
            combineFhistFiles(fileparts(matlabFileName), fhistFileName);
        end
        return;
    end
end


%Display Chirp parameters in table on screen
function h = displayChirpParams(Params, Position, Parent)

    dat =  {'Start Frequency (Ghz)', Params.dataPath.startFreq;...
            'Slope (MHz/us)', Params.dataPath.freqSlopeConst;...   
            'Samples per chirp', Params.dataPath.numAdcSamples;...
            'Chirps per frame',  Params.dataPath.numChirpsPerFrame;...
            'Frame duration (ms)',  Params.frameCfg.framePeriodicity;...
            'Sampling rate (Msps)', Params.dataPath.digOutSampleRate / 1000;...
            'Bandwidth (GHz)', Params.dataPath.freqSlopeConst * Params.dataPath.numAdcSamples /...
                               Params.dataPath.digOutSampleRate;...
            'Range resolution (m)', Params.dataPath.rangeResolutionMeters;...
            'Velocity resolution (m/s)', Params.dataPath.dopplerResolutionMps;...
            'Number of Rx (MIMO)', Params.dataPath.numRxAnt; ...
            'Number of Tx (MIMO)', Params.dataPath.numTxAnt;};
    columnname =   {'Chirp Parameter (Units)', 'Value'};
    columnformat = {'char', 'numeric'};
    
    h = uitable('Units','normalized', ...
            'Parent', Parent, ...
            'Position', Position, ...
            'Data', dat,... 
            'ColumnName', columnname,...
            'ColumnFormat', columnformat,...
            'ColumnWidth', 'auto',...
            'RowName',[]);
end

function exitPressFcn(hObject, ~)
    setappdata(hObject, 'exitKeyPressed', 1);
end

function sphandle = configureDataPort(comPortNum,baudrate)
    comPortsAvailable = serialportlist("available");
    comPortString = ['COM' num2str(comPortNum)];
    if any(contains(comPortsAvailable,comPortString))
        sphandle = serialport(comPortString,baudrate,'Parity','none','Timeout',10); 
        configureTerminator(sphandle,0);
        flush(sphandle);
    else
        sphandle = [];
        fprintf('Serial port %s is already open or not available! Power cycle the device and re-run the application...\n', comPortString);
    end
end

function sphandle = configureControlPort(comPortNum,baudrate,terminator)
    comPortsAvailable = serialportlist("available");
    comPortString = ['COM' num2str(comPortNum)];
    if any(contains(comPortsAvailable,comPortString))
        sphandle = serialport(comPortString,baudrate,'Parity','none','Timeout',10); 
        configureTerminator(sphandle,terminator);
        flush(sphandle);
    else
        sphandle = [];
        fprintf('Serial port %s is already open or not available! Power cycle the device and re-run the application...\n', comPortString);
    end
end

function sphandle = reconfigureControlPort(sphandle)
    configureTerminator(sphandle,0);
end

function setSensorAxesFigure(ax, sensor, sensorMount, color)
if nargin<4, color = 'k'; end

% Rotation variables
azimuthTilt = sensorMount.azimuthTilt;
yawTilt = sensorMount.yawTilt;
elevationTilt = sensorMount.elevationTilt;

% Boresight Line
boresightLineEnd = rotatePoint([0;sensor.rangeMax;0], azimuthTilt, elevationTilt, yawTilt);
line(ax,[0 boresightLineEnd(1)]+sensorMount.sensorPos(1),...
    [0 boresightLineEnd(2)]+sensorMount.sensorPos(2),...
    [0 boresightLineEnd(3)]+sensorMount.sensorPos(3),'Marker','o','Color',color,'LineStyle','--');

% Azimuth FOV Lines
azimFovLineLeftEnd  = rotatePoint(rotatePoint([0;sensor.rangeMax;0],sensor.azim(end),0,0), azimuthTilt, elevationTilt, yawTilt);
azimFovLineRightEnd = rotatePoint(rotatePoint([0;sensor.rangeMax;0],sensor.azim(1),0,0), azimuthTilt, elevationTilt, yawTilt);
line(ax,[0 azimFovLineLeftEnd(1)]+sensorMount.sensorPos(1), ...
    [0 azimFovLineLeftEnd(2)]+sensorMount.sensorPos(2), ...
    [0 azimFovLineLeftEnd(3)]+sensorMount.sensorPos(3), 'Color',color,'LineStyle','--');
line(ax,[0 azimFovLineRightEnd(1)]+sensorMount.sensorPos(1), ...
    [0 azimFovLineRightEnd(2)]+sensorMount.sensorPos(2), ...
    [0 azimFovLineRightEnd(3)]+sensorMount.sensorPos(3), 'Color',color,'LineStyle','--');

% Elevation FOV Lines
elevFovLineUpEnd   = rotatePoint(rotatePoint([0;sensor.rangeMax;0],0,sensor.elev(end),0), azimuthTilt, elevationTilt, yawTilt);
elevFovLineDownEnd = rotatePoint(rotatePoint([0;sensor.rangeMax;0],0,sensor.elev(1),0), azimuthTilt, elevationTilt, yawTilt);
line(ax,[0 elevFovLineUpEnd(1)]+sensorMount.sensorPos(1), ...
    [0 elevFovLineUpEnd(2)]+sensorMount.sensorPos(2), ...
    [0 elevFovLineUpEnd(3)]+sensorMount.sensorPos(3), 'Color',color,'LineStyle','--');
line(ax,[0 elevFovLineDownEnd(1)]+sensorMount.sensorPos(1), ...
    [0 elevFovLineDownEnd(2)]+sensorMount.sensorPos(2), ...
    [0 elevFovLineDownEnd(3)]+sensorMount.sensorPos(3), 'Color',color,'LineStyle','--');

end

function posRT = rotatePoint(points,azim,elev,yaw)
    if nargin<4, yaw = 0; end
    % Used the same rotations: https://www.mathworks.com/help/phased/ref/rotx.html
    % rotx = elev;      counterclockwise (+)
    % roty = yaw;       counterclockwise (+)
    % rotz = azim;      counterclockwise (+)
    
    Rot_TW = getRotationMatrix(elev,yaw,azim);
    posRT = Rot_TW*points;
end

function Rot_TW = getRotationMatrix(rotx_tw,roty_tw,rotz_tw)
% Used the same rotations: https://www.mathworks.com/help/phased/ref/rotx.html
RotX_TW = [1 0 0; 0 cos(rotx_tw) -sin(rotx_tw); 0 sin(rotx_tw) cos(rotx_tw)];
RotY_TW = [cos(roty_tw) 0 sin(roty_tw); 0 1 0; -sin(roty_tw) 0 cos(roty_tw)];
RotZ_TW = [cos(rotz_tw) -sin(rotz_tw) 0; sin(rotz_tw) cos(rotz_tw) 0; 0 0 1];
Rot_TW  = RotZ_TW * RotY_TW * RotX_TW;
end

function config = readCfg(filename)
    config = cell(1,100);
    fid = fopen(filename, 'r');
    if fid == -1
        fprintf('File %s not found!\n', filename);
        return;
    else
        fprintf('Opening configuration file %s ...\n', filename);
    end
    tline = fgetl(fid);
    k=1;
    while ischar(tline)
        config{k} = tline;
        tline = fgetl(fid);
        k = k + 1;
    end
    config = config(1:k-1);
    fclose(fid);
end

function box_out = convertBoxFormat(box_in)
x_orig = box_in(1);
y_orig = box_in(3);
z_orig = box_in(5);
x_dim = box_in(2) - box_in(1);
y_dim = box_in(4) - box_in(3);
z_dim = box_in(6) - box_in(5);
box_out = [x_orig, y_orig, z_orig, x_dim, y_dim, z_dim];
end

function handle = plotCube(ax, origin, dim, color, linestyle, linewidth, facecolor, facealpha)
if nargin<6, linewidth = 0.3; end
if nargin<7, facecolor = 'none'; end
if nargin<8, facealpha = 0.5; end

% Define the vertexes of the unit cubic
ver = [1 1 0; 0 1 0; 0 1 1; 1 1 1; 0 0 1; 1 0 1; 1 0 0; 0 0 0];
%  Define the faces of the unit cubic
fac = [1 2 3 4; 4 3 5 6; 6 7 8 5; 1 2 8 7; 6 7 1 4; 2 3 5 8];

cube = [ver(:,1)*dim(1)+origin(1),ver(:,2)*dim(2)+origin(2),ver(:,3)*dim(3)+origin(3)];
handle = patch(ax, 'Faces',fac,'Vertices',cube,'EdgeColor',color,'LineStyle',linestyle,'LineWidth',linewidth,'FaceColor',facecolor,'FaceAlpha',facealpha);
end

function configureDisabledWindow(ax,message)
if nargin<2, message = 'Disabled in the cfg file!'; end
set(ax,'XColor','none','YColor','none');
xPos = 0; yPos = 0.5;
set(ax,'Color',0.7*[1 1 1])
text(ax,xPos,yPos,message,'FontSize',18,'Color','w');
end

% Prepare for point cloud for visualization
function [point3Dw_plot, classifierBlkData, currBlkIdx] = pointCloudVisualization(point3Dw, pointCloud, scene, visulizePointCloud, config, classifierBlkData, currBlkIdx)
% Extract configuration parameters
xyOffsetFlag = config.xyOffsetFlag;
dbScanFlag = config.dbScanFlag;
dbScanEpsilon = config.dbScanEpsilon;
dbScanMinPts = config.dbScanMinPts;

% Prepare the detected object
detectObjCur.nDetObj = size(point3Dw,2);
detectObjCur.zone_info = zoneAssign(detectObjCur.nDetObj,point3Dw,scene.totNumZone,scene.zoneDef);
detectObjCur.snr = pointCloud(5,:);
detectObjCur.doppler = pointCloud(4,:);
detectObjCur.x_cord = point3Dw(1,:);
detectObjCur.y_cord = point3Dw(2,:);
detectObjCur.z_cord = point3Dw(3,:);

% Run the feature block
[~, classifierBlkData, currBlkIdx, ...
    x_cord_out, y_cord_out, z_cord_out, x_cord_blk_out, y_cord_blk_out, z_cord_blk_out, ...
    ] = featureExtractionPerFrame(detectObjCur, scene, xyOffsetFlag, dbScanFlag, dbScanEpsilon, dbScanMinPts, classifierBlkData, currBlkIdx);

% Chose the visualation option
switch visulizePointCloud
    case 1
        x_cord_vis = [x_cord_out{:}];
        y_cord_vis = [y_cord_out{:}];
        z_cord_vis = [z_cord_out{:}];
    case 2
        x_cord_vis = [x_cord_blk_out{:}];
        y_cord_vis = [y_cord_blk_out{:}];
        z_cord_vis = [z_cord_blk_out{:}];
end
point3Dw_plot = [x_cord_vis; y_cord_vis; z_cord_vis];
end

% Returns the folder where the compiled executable actually resides.
function executableFolder = preparePath(visulizePointCloud)
try
    if isdeployed
        % User is running an executable in standalone mode.
        [~, result] = system('set PATH');
        executableFolder = char(regexpi(result, 'Path=(.*?);', 'tokens', 'once'));
    else
        % User is running an m-file from the MATLAB integrated development environment (regular MATLAB).
        executableFolder = pwd;

        % Add Paths
        addpath(genpath([executableFolder '\utilityFiles']));
        
        if (visulizePointCloud > 0)
            addpath(genpath(strrep(executableFolder,'demo_target\demo_gui','matlab_highacc_sbr_cpd\classifier\featureExtraction')));
        end
    end
catch ME
    errorMessage = fprintf('Error in function %s() at line %d.\n\nError Message:\n%s', ...
        ME.stack(1).name, ME.stack(1).line, ME.message);
    uiwait(warndlg(errorMessage));
end
end

% Combine all the fHist files in the save folder
function combineFhistFiles(dataFolderName, dataPrefix)
    % Create data store
    fds = fileDatastore(dataFolderName, 'ReadFcn', @readFcn, 'IncludeSubfolders', false, 'FileExtensions', '.mat');
    
    % Statistics
    numFiles = 0;
    numFrames = 0;
    isFirstFile = 1;
    
    % Read and process all the files
    while hasdata(fds)
        % Get the next file
        inputFileName = read(fds);
    
        % Parse the file name
        [~,fileName,~] = fileparts(inputFileName);
        
        % Process data if starts with the desired pattern
        nameStrPattern = string(dataPrefix) + "_" + digitsPattern(4);
        fileName = extract(fileName,nameStrPattern);
        if isempty(fileName)
            continue;
        end
    
        % Update the file statistics
        numFiles = numFiles + 1;
    
        % Load the data
        dataPerFile = load(inputFileName);
        dataFields = fields(dataPerFile);
    
        % Analyze the fields
        combineFieldIdx = zeros(1,length(dataFields));
        frameIdx = zeros(1,length(dataFields));
        numFramesPerField = zeros(1,length(dataFields));
        for idx = 1:length(dataFields)
            fieldName = dataFields{idx};
            dataPerField = dataPerFile.(fieldName);
            if size(dataPerField,ndims(dataPerField)) > 1
                combineFieldIdx(idx) = 1;
                frameIdx(idx) = ndims(dataPerField);
                numFramesPerField(idx) = size(dataPerField,ndims(dataPerField));
            else
                combineFieldIdx(idx) = 0;
            end
        end
    
        % Update the frame statistics
        numFramesInFields = unique(numFramesPerField);
        if (length(numFramesInFields)<=2)
            numFrames = numFrames + numFramesInFields(numFramesInFields~=0);
        end
    
        % Combine data
        if isFirstFile
            dataCombined = dataPerFile;
            isFirstFile = 0;
        else
            for idx = 1:length(dataFields)
                fieldName = dataFields{idx};
                if (combineFieldIdx(idx) == 1)
                    dataCombined.(fieldName) = cat(frameIdx(idx),dataCombined.(fieldName),dataPerFile.(fieldName));
                end
            end
        end
    
        % Delete the complete fHist file
        delete(inputFileName);

        % Status screen
        statusText = ['Combined ' '''' num2str(numFiles) '''' ' files with total ' '''' num2str(numFrames) '''' ' frames...'];
        disp(statusText)
    end
    
    % Save the data
    fileNameSave = fullfile(dataFolderName, [dataPrefix, '_0000', '.mat']);
    disp(['Saving data to ', fileNameSave, ' ...']);
    save(fileNameSave,'-struct','dataCombined')
    
    % Datastore read function
    function data = readFcn(filename)
        data = filename;
    end
end