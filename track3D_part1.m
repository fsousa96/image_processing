function [] = track3D_part1(imgseq1, cam_params)

R = cam_params.R;
T = cam_params.T;


%% Perform background subtraction and apply the camera-to-world tranformation and the camera intrinsic parameters
nr_frames = size(imgseq1.rgb,3);

bg_depth = median(imgseq1.depth,3);
bg_gray = median(imgseq1.rgb,3);
%figure(1);
%subplot(211);imagesc(bgdepth);
%subplot(212);imagesc(bggray);

% figure(1);clf;
% figure(2);clf;
 for i=1:nr_frames
     imdiff = imgseq1.depth(:,:,i) > 0 & abs(imgseq1.depth(:,:,i) - bg_depth)>.20;
     imgdiffiltered(:,:,i) = imopen(imdiff,strel('disk',5));
     %figure(1);
    % imagesc([imdiff imgdiffiltered]);
     %title('Difference image and morph filtered');
     %colormap(gray);
     %figure(2);
     %imagesc([imgseq1.depth(:,:,i) bg_depth]);
     %title('Depth image i and background image');
     %figure(3);
     connected_components = bwlabel(imgdiffiltered(:,:,i));
     s = regionprops(connected_components, 'centroid');
     a = regionprops(connected_components, 'area');
     figure(1);
     imagesc(bwlabel(imgdiffiltered(:,:,i)));
     title('Connected components');

%      if (i == 6)
%      figure(5)
%      imagesc(bwlabel(imgdiffiltered(:,:,i)));
%      title('Connected components');
%      end
%      
%      if (i == 5)
%      figure(2)
%      imagesc(bwlabel(imgdiffiltered(:,:,i)));
%      title('Connected components - previous');
%      end
     
     
     for k=1:size(s,1) % for every region 
         centroid = s(k).Centroid;
         area = a(k).Area;
         id(k) = k;
     end
     
     centroids = [s.Centroid];
     areas = [a.Area];

     centroidX = centroids(1:2:end-1);
     centroidY = centroids(2:2:end);
     measurements(i).areas = areas';
     measurements(i).centroids = [centroidX' centroidY'];
     measurements(i).ids = id;
     
     for k=1:size(s,1)
        text(centroidX(k), centroidY(k), num2str(k), 'FontSize', 14, 'FontWeight', 'Bold'); 
        measurements(i).centroids(k,3) = k;
     end
     %hold on
     %plot(centroids(:,1), centroids(:, 2), 'b*');
     %hold off
     %title('Connected components');
     %pause;
     
 end
 
 for i=4:nr_frames
      %  current = imgdiffiltered(i);
       % previous = imgdiffiltered(i-1);
        %size(measurements(6).centroids,1);
        for a=1:size(measurements(i).centroids,1)
            for b=1:size(measurements(i-1).centroids,1)
             X = [measurements(i).centroids(a, 1),measurements(i).centroids(a, 2); measurements(i-1).centroids(b, 1), measurements(i-1).centroids(b, 2)];
             distances(a, b) = pdist(X, 'euclidean');
            end
            [value, arg] = min(distances(a,:));
            id(a) = arg;
            size(id, 1);
            measurements(i).centroids(a,3) = id(a);
            
        end
       
 end
 
  assignin('base', 'distances', distances);
 assignin('base', 'id', id);
 assignin('base','measurements', measurements);
 
 %%check for movement
 for i=3:nr_frames
    %find( measurements(i).centroids(:, 3), 1)
    if(i == 3)
     index = 1;
    end
    
     object = measurements(i).ids(:,find(measurements(i).centroids(:, 3) == index))
     %escolher o centroid mais proximo
     index = object
     
 end
 

 
 
 

end

