function writeLineSerial(sphandle, cliCfg)
if sphandle.BaudRate > 115200
    writelineslow(sphandle, cliCfg, 0.002);
else
    writelineslow(sphandle, cliCfg, 0.0005);
end
end

% Utility function
function writelineslow(sphandle, cliCfg, timeout)
terminator = sphandle.Terminator;
configureTerminator(sphandle,0);
for n = 1:length(cliCfg)
    if n~=length(cliCfg)
        write(sphandle,cliCfg(n),"char")
        pause(timeout);
    else
        configureTerminator(sphandle,terminator);
        writeline(sphandle,cliCfg(n));
    end
end
end