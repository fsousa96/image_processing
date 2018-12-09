close all;
clear;
load('cameraparametersAsus.mat');
%load('depth_10.mat');

D = 'filinha';
jpg1 = dir(fullfile(D, 'images*.jpg'));
mat1 = dir(fullfile(D, 'images*.mat'));
%d=dir(fullfile(D,'*.jpg'));
%dd=dir(fullfile(D,'*.mat'));
% d = dir('images*.jpg');
% dd = dir('images*.mat');

% a = zeros(480,640,len7gth(d));
% b = zeros(480, 640, length(d));
for i=1:numel(jpg1)    
    name1 = fullfile(D, jpg1(i).name);  
    data1 = fullfile(D, mat1(i).name);
    imgseq1(i).rgb = imread(name1);
    load(data1);
    imgseq1(i).depth = depth_array;
%     figure(1)
%     imshow(uint8(imgseq1(i).rgb));
%     figure(2);
%     imagesc(imgseq1(i).depth);
%     colormap(gray);
%     pause(0.1);
end

 track3D_part1(imgseq1, cam_params);