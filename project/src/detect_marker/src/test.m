clc;
clear; close all;

blurred = double(rgb2gray(imread('blurred_low.png'))) / 255;

PSF = fspecial('motion', 12, 0);
l = size(PSF, 2);
m = fix(l/2)+1;
PSF(:, 1:m-1) = 0;
PSF = PSF / sum(PSF(:))

restored = deconvlucy(blurred, PSF);

k = 4;
x = 323;
y = 181;
[h,w] = size(restored);
restored = imresize(restored(y:y+h/k, x:x+w/k), k, 'nearest');
blurred = imresize(blurred(y:y+h/k, x:x+w/k), k, 'nearest');

imshow(restored);
title('Restored Image');

restored = restored >= 0.7;
restored = imopen(restored, ones(1,16)) | imopen(restored, ones(16,1));

figure;
imshow(blurred);
title('Original Image');
figure;
imshow(restored);
title('Restored Image');

imwrite(restored, 'restored.png');
