%%
d=dir('*.jpg');
dd=dir('*.mat');
imgs=zeros(480,640,length(d));
imgsd=zeros(480,640,length(d));
for i=1:length(d),
    imgs(:,:,i)=rgb2gray(imread(d(i).name));
    load(dd(i).name);
    imgsd(:,:,i)=double(depth_array)/1000;
    %imread(d(i).name);
    %xyz1=get_xyzasus(depth_array(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    %figure(1)
    %imshow(uint8(imgs(:,:,i)));
    %figure(2);
    %imagesc(imgsd(:,:,i));
    %colormap(gray);
    %pause(0.1);
end
%%
bgdepth=median(imgsd,3);
bggray=median(imgs,3);
figure(1);
subplot(211);imagesc(bgdepth);
subplot(212);imagesc(bggray);
%%
%Bg subtraction for depth (try with gray too)
figure(1);clf;
figure(2);clf;
for i=1:length(d)
    imdiff=abs(imgsd(:,:,i)-bgdepth)>.20;
    imgdiffiltered=imopen(imdiff,strel('disk',5));
    figure(1);
    imagesc([imdiff imgdiffiltered]);
    title('Difference image and morph filtered');
    colormap(gray);
    figure(2);
    imagesc([imgsd(:,:,i) bgdepth]);
    title('Depth image i and background image');
    figure(3);
    imagesc(bwlabel(imgdiffiltered));
    title('Connected components');
    pause(0.1);
    
end