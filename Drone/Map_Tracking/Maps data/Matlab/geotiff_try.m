% Define a matrix of size NxMx3 (representing an RGB image)
N = 500;  % height
M = 700;  % width
imageData = uint8(randi([0, 255], N, M, 3));  % Random RGB image data

% Create a map raster reference object for georeferencing (spatial referencing)
% You can modify the extent of the image in real-world coordinates if you have this info.
R = maprefcells([-90 90], [-180 180], size(imageData(:,:,1)));  % Adjust these coordinates to fit your map's extent

% Optionally, specify the coordinate reference system (CRS) code for projection
% For example, 'EPSG:4326' is the CRS code for WGS 84.
crsCode = 4326;  % This is just an example (use appropriate for your case)

% Specify output file name
outputFile = 'output_rgb_map.tif';

% Save the imageData matrix as a GeoTIFF with georeferencing
geotiffwrite(outputFile, imageData, R, 'CoordRefSysCode', crsCode);

disp('GeoTIFF file written successfully.');
