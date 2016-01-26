clc;
clear; close all;

ref = double(rgb2gray(imread('ref.png'))) / 255;
blurred_high = double(rgb2gray(imread('blurred_high.png'))) / 255;
blurred_low = double(rgb2gray(imread('blurred_low.png'))) / 255;

f = imfilter(ref, fspecial('prewitt')');
s1 = f(:)'*f(:);
f = imfilter(blurred_low, fspecial('prewitt')');
s2 = f(:)'*f(:);
f = imfilter(blurred_high, fspecial('prewitt')');
s3 = f(:)'*f(:);
