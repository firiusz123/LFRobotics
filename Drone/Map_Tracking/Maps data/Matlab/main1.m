% Specify the path to your GeoTIFF file
filePath = 'krk_bigger_12008.tif';  % Replace with your file path

% Read the GeoTIFF data and the spatial referencing information
[data, R] = geotiffread(filePath);
%%
% Display the size of the data (if it's a multi-band image)
disp('Size of GeoTIFF data:');
disp(size(data));
image_data = data(:,:,1:3);
% Display the GeoTIFF image
figure;
imshow(image_data);
%title('GeoTIFF Image');
%%
% If the image has multiple bands, display the first band
if ndims(data) == 3
    figure;
    imshow(data(:,:,1), []);
    title('Band 1 of GeoTIFF');
end
%%
% Show the georeferenced image using map coordinates
figure;
mapshow(data(:,:,1:3), R);
title('Georeferenced GeoTIFF Image');
%%
% Read the metadata of the GeoTIFF file
info = geotiffinfo(filePath);

% Display metadata information
disp('GeoTIFF Metadata:');
disp(info);

% Display the spatial referencing information (coordinate system, scale, etc.)
disp('Spatial Referencing Information:');
disp(R);
