close all;
clear;
load('cameraparametersAsus.mat');
%load('depth_10.mat');

D1 = 'imgseq1';
D2 = 'imgseq2';
jpg1 = dir(fullfile(D1, 'images*.jpg'));
mat1 = dir(fullfile(D1, 'images*.mat'));
jpg2 = dir(fullfile(D2, 'images*.jpg'));
mat2 = dir(fullfile(D2, 'images*.mat'));
%d=dir(fullfile(D,'*.jpg'));
%dd=dir(fullfile(D,'*.mat'));
% d = dir('images*.jpg');
% dd = dir('images*.mat');

% a = zeros(480,640,length(d));
% b = zeros(480, 640, length(d));
for i=1:numel(jpg1)    
    name1 = fullfile(D1, jpg1(i).name);  
    data1 = fullfile(D1, mat1(i).name);
    name2 = fullfile(D2, jpg2(i).name);  
    data2 = fullfile(D2, mat2(i).name);
    imgseq1(i).rgb = imread(name1);
    load(data1);
    imgseq1(i).depth = depth_array;
    imgseq2(i).rgb = imread(name2);
    load(data2);
    imgseq2(i).depth = depth_array;
    %figure(1)
    %imshow(uint8(imgseq1(:,:,i).rgb));
    %figure(2);
    %imagesc(imgseq1(:,:,i).depth);
    %colormap(gray);
    %pause(0.1);
end

track3D_part2(imgseq1, imgseq2, cam_params);