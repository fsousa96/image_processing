close all;
clear;
load('cameraparametersAsus.mat');
load('depth_10.mat');

d=dir('*.jpg');
dd=dir('*.mat');

a = zeros(480,640,length(d));
b = zeros(480, 640, length(d));
for i=1:length(d)    
    a(:,:,i) = rgb2gray(imread(d(i).name));
    load(dd(i).name);
    b(:,:,i) = double(depth_array)/1000;
    %figure(1)
    %imshow(uint8(imgseq1(:,:,i).rgb));
    %figure(2);
    %imagesc(imgseq1(:,:,i).depth);
    %colormap(gray);
    %pause(0.1);
end
imgseq1.depth = b;
imgseq1.rgb = a;

track3D_part1(imgseq1, cam_params);