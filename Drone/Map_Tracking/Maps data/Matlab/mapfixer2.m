% Specify the input and output file names
mainMapFile = 'krk_bigger_12008.tif';          % Main map file
replacementMapFile = 'krk_bigger_11008.tif';   % Replacement map file
outputFile = 'output_map.tif';                 % Output file name

% Read the main map and replacement map
[mainData, R] = readgeoraster(mainMapFile);
[replacementData, R1] = readgeoraster(replacementMapFile);

% Get GeoTIFF metadata to extract CRS information
info = geotiffinfo(mainMapFile);

% Extract CRS (EPSG) code from GeoTIFF metadata
if isfield(info.GeoTIFFCodes, 'PCS')
    crsCode = info.GeoTIFFCodes.PCS;  % Projected Coordinate System (EPSG code)
else
    crsCode = 4326;  % Default to WGS84 if no CRS is available
end
%%
% Ensure the replacement map is in uint8 format
replacementData = uint8(replacementData);  % Restrict to first 3 channels (RGB)
mainDataTemp = uint8(mainData);  % Restrict to first 3 channels (RGB)

% Replace 0 values in the main map with corresponding values from the replacement map
mainDataTemp(mainDataTemp == 0) = replacementData(mainDataTemp == 0);
mainDataTemp(mainDataTemp == 0) = 255;

% Display the modified map
figure;
imshow(mainDataTemp(:,:,1:3));  % Ensure RGB format for display
title('Modified Map');
%%
% Save the modified map to a GeoTIFF file with CRS information
geotiffwrite(outputFile, mainDataTemp, R, 'CoordRefSysCode', crsCode);
%%
outputFile = 'output_map.tif';                 
% Output file name
% Read the main map and replacement map
[mainData, R] = readgeoraster(outputFile);

imshow(mainData)


