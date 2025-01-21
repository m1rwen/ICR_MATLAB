function [hControlSerialPort, sensorStartCmd] = sendCfgToDevice(hControlSerialPort, cliCfg)
sensorStartCmd = 'sensorStart'; % Default sensor start command

for k=1:length(cliCfg)
    if isempty(strrep(strrep(cliCfg{k},char(9),''),char(32),''))
        continue;
    end
    if strcmp(cliCfg{k}(1),'%')
        continue;
    end
    if startsWith(cliCfg{k},'sensorStart')
        % Wait to start the sensor
        sensorStartCmd = cliCfg{k};
        continue;
    end

    writeLineSerial(hControlSerialPort, cliCfg{k});
    fprintf('%s\n', cliCfg{k});

    if strcmp('baudRate',strtok(cliCfg{k},' '))
        [~, baudRate] = strtok(cliCfg{k},' ');
        set(hControlSerialPort, 'BaudRate', str2double(baudRate));
        pause(.5);
        continue;
    end

    for kk = 1:3
        cc = readline(hControlSerialPort);
        if contains(cc, 'Done')
            fprintf('%s\n',cc);
            break;
        elseif contains(cc, 'not recognized as a CLI command')
            fprintf('%s\n',cc);
        elseif contains(cc, 'Debug:')
            fprintf('%s\n',cc);
        elseif contains(cc, 'Error')
            fprintf('%s\n',cc);
            return;
        end
    end
end