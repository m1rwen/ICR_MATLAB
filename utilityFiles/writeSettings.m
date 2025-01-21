function writeSettings(filename, configPathName, configFileName)
    fid = fopen(filename, 'w');
    if fid == -1
        fprintf('File %s not found!\n', filename);
        return;
    end

    fprintf(fid,'configPathName\t%s\n',configPathName);
    fprintf(fid,'configFileName\t%s',configFileName);
    
    fclose(fid);
end