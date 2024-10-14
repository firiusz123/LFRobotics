% Specify the input and output file names
mainMapFile = 'krk_bigger_12008.tif';          % Main map file
replacementMapFile = 'krk_bigger_11008.tif';  % Replacement map file

outputFile = 'output_map.tif';         % Output file name

% Read the main map and replacement map
[mainData, R] = readgeoraster(mainMapFile);
[replacementData, R1]= readgeoraster(replacementMapFile);


info = geotiffinfo(mainMapFile)



%%
replacementData = uint8(replacementData);
%end
mainDataTemp = uint8(mainData);
% Replace 0 values in main map with corresponding values from the replacement map
mainDataTemp(mainDataTemp == 0) = replacementData(mainDataTemp == 0);
%%
% Display the modified map
figure;
imshow(mainDataTemp(:,:,1:3));
title('Modified Map');

%%

geotiffwrite(outputFile, mainDataTemp, R)
%%
geotiffwrite(outputFile, mainDataTemp(:,:,1:3), R , crsCode);


