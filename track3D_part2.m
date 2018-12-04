function [ ] = track3D_part2( imgseq1, imgseq2, cam_params )


load('CalibrationData.mat');
nr_frames = size(imgseq1,2);

for i = 1:nr_frames
    
   imgs1(:, :, i) = rgb2gray(imgseq1(i).rgb);
   imgsd1(:, :, i) = double(imgseq1(i).depth)/1000;
   imgs2(:, :, i) = rgb2gray(imgseq2(i).rgb);
   imgsd2(:, :, i) = double(imgseq2(i).depth)/1000;
   
end

bg_depth1 = median(imgsd1,3);
bg_gray1 = median(imgs1,3);
bg_depth2 = median(imgsd2,3);
bg_gray2 = median(imgs2,3);


keypoints_cam1 = [];
keypoints_cam2 = [];

for i = 1:nr_frames
    xyz_sift = sift_to_3d(imgseq1(i).rgb, imgseqfg2(i).rgb, imgseqfg1(i).depth, imgseqfg2(i).depth, cam_params.Kdepth);
end

end

