function [configPathName, configFileName] = readSettings(filename)
    setting = cell(1,2);
    fid = fopen(filename, 'r');
    if fid == -1
        fprintf('File %s not found!\n', filename);
        configPathName = []; configFileName = [];
        return;
    end
    
    for k = 1:length(setting)
        setting{k} = fgetl(fid);
    end
    fclose(fid);

    tline = strsplit(setting{1});
    configPathName = strtrim(extractAfter(setting{1},tline{1}));

    tline = strsplit(setting{2});
    configFileName = strtrim(extractAfter(setting{2},tline{1}));
    
end