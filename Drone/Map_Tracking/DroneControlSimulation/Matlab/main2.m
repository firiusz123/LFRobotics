% Specify the input and output GeoTIFF file names
inputFileName = 'krk_bigger_12008.tif';  % Replace with your input file
outputFileName = 'output_file.tif'; % Name for the output file

% Read the GeoTIFF file
[data, R] = geotiffread(inputFileName);

data1 = mean(data(:,:,1:3), 3);
% Convert to uint8 for 8-bit representation
data1 = uint8(data1);

% Display the original data for reference
figure;
imshow(data1, []);
title('Original Image');
%%
% Set No Data values to 255 (white)
noDataValue = 0;  % Specify your No Data value here
data1(data1 == noDataValue) = 255;  % Change No Data values to white

% Display the modified data
figure;
imshow(data1, []);
title('Modified Image with No Data set to White');

% Write the modified data back to a GeoTIFF file
% Update metadata as needed; for simplicity, we use original metadata
geotiffwrite(outputFileName, data1, R, 'CoordRefSysCode', R.CoordRefSysCode, 'GeoKeyDirectoryTag', R.GeoKeyDirectoryTag);
