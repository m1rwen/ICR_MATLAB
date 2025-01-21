% Find COM port numbers for control and data serial ports
function [controlPortNum, dataPortNum] = findDemoComPorts()
    % Initialize the ports to default
    controlPortNum = [];
    dataPortNum = [];

    % List all the USB devices
    devices = IDSerialComs();
    if isempty(devices)
        return;
    end
    devicesNames = devices(:,1);
    portNumbers = devices(:,2);

    % Com Port names
    CLI_XDS_SERIAL_PORT_NAME = 'XDS110 Class Application/User UART';
    DATA_XDS_SERIAL_PORT_NAME = 'XDS110 Class Auxiliary Data Port';
    CLI_SIL_SERIAL_PORT_NAME = 'Enhanced COM Port';
    DATA_SIL_SERIAL_PORT_NAME = 'Standard COM Port';

    % Find the user port
    offsName = cellfun(@(c) strfind(c, CLI_XDS_SERIAL_PORT_NAME), devicesNames, 'UniformOutput', false);
    portIdx = find(not(cellfun('isempty',offsName)), 1);
    if isempty(portIdx)
        offsName = cellfun(@(c) strfind(c, CLI_SIL_SERIAL_PORT_NAME), devicesNames, 'UniformOutput', false);
        portIdx = find(not(cellfun('isempty',offsName)), 1);
    end
    if ~isempty(portIdx)
        portNum = portNumbers{portIdx};
        if isnumeric(portNum)
            controlPortNum = portNum;
        else
            return;
        end
    else
        return;
    end
        
    % Find the data port
    offsName = cellfun(@(c) strfind(c, DATA_XDS_SERIAL_PORT_NAME), devicesNames, 'UniformOutput', false);
    portIdx = find(not(cellfun('isempty',offsName)), 1);
    if isempty(portIdx)
        offsName = cellfun(@(c) strfind(c, DATA_SIL_SERIAL_PORT_NAME), devicesNames, 'UniformOutput', false);
        portIdx = find(not(cellfun('isempty',offsName)), 1);
    end
    if ~isempty(portIdx)
        portNum = portNumbers{portIdx};
        if isnumeric(portNum)
            dataPortNum = portNum;
        else
            return;
        end
    else
        return;
    end
end



function devices = IDSerialComs()
% IDSerialComs identifies Serial COM devices on Windows systems by friendly name
% Searches the Windows registry for serial hardware info and returns devices,
% a cell array where the first column holds the name of the device and the
% second column holds the COM number. Devices returns empty if nothing is found.

% Check the available COM ports
devices = [];
coms = cellstr(serialportlist("available"));

key = 'HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\USB\';
[~, vals] = dos(['REG QUERY ' key ' /s /f "FriendlyName" /t "REG_SZ"']);
if ischar(vals) && strcmp('ERROR',vals(1:5))
    disp('Error: IDSerialComs - No Enumerated USB registry entry')
    return;
end
vals = textscan(vals,'%s','delimiter','\t');
vals = cat(1,vals{:});
out = 0;
for i = 1:numel(vals)
    if strcmp(vals{i}(1:min(12,end)),'FriendlyName')
        if ~iscell(out)
            out = vals(i);
        else
            out{end+1} = vals{i}; %#ok<AGROW> Loop size is always small
        end
    end
end

for i = 1:numel(coms)
    match = strfind(out,[coms{i},')']);
    ind = 0;
    for j = 1:numel(match)
        if ~isempty(match{j})
            ind = j;
        end
    end
    if ind ~= 0
        com = str2double(coms{i}(4:end));
        if com > 9
            length = 8;
        else
            length = 7;
        end
        devices{i,1} = out{ind}(27:end-length); %#ok<AGROW>
        devices{i,2} = com; %#ok<AGROW> Loop size is always small
    end
end
end