function [] = track3D_part1(imgseq1, cam_params)

R = cam_params.R;
T = cam_params.T;


%% Perform background subtraction and apply the camera-to-world tranformation and the camera intrinsic parameters
nr_frames = size(imgseq1.rgb,3);

bg_depth = median(imgseq1.depth,3);
bg_gray = median(imgseq1.rgb,3);
% figure(10);
% subplot(211);imagesc(bg_depth);
% subplot(212);imagesc(bg_gray);

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
      
%      figure(i);
%      imagesc(bwlabel(imgdiffiltered(:,:,i)));
%      title('Connected components');

%      if (i == 5)
%      figure(i)
%      imagesc(bwlabel(imgdiffiltered(:,:,i)));
%      title('Connected components');
%      end
%      
%      if (i == 4)
%      figure(i)
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
     
     if(i == 5 || i == 4)
        for k=1:size(s,1)
            text(centroidX(k), centroidY(k), num2str(k), 'FontSize', 14, 'FontWeight', 'Bold'); 
            measurements(i).centroids(k,3) = k;
        end
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
             distances(a, b, i) = pdist(X, 'euclidean');
%             abs(measurements(i).areas(a) - measurements(i-1).areas(b))
             areas(a, b, i) = abs(measurements(i).areas(a)/measurements(i-1).areas(b));
             %cost(a, b, i) = distances(a, b, i);% - areas(a, b, i);
             %if(distances(a, b) > 60)
              %   distances(a, b) = NaN;
             %end
            end
            [value, arg] = min(distances(a, :, 4));
            id(a) = arg;%returns the index of the closest centroid of the previous frame
            measurements(i).centroids(a,3) = id(a);
            
        end
               
 end
 for i=1:size(distances, 3)
     for a=1:size(distances, 2)
         for b=1:size(distances, 1)            

                 distances(a, b, i) = distances(a, b, i) / 50;
              cost_function(a, b, i) = abs(areas(a, b, i) - 1)*0.75 + abs(distances(a, b, i) - 1)*0.25; 
         end

     end
      cost_function(:,:,i) = cost_function(:,:,i).';
 end

 assignin('base', 'distances', distances);
 assignin('base', 'areas', areas);
 assignin('base', 'id', id);
 assignin('base','measurements', measurements);
 assignin('base','cost_function', cost_function);

 
 %% check for movement
 
 for i=4:nr_frames

     [assignment, cost] = HungarianMethod(cost_function(:, :, i));
    % index = assignment(index);
    
    for a=1:length(assignment)
        costs = cost_function(a,assignment(a),i);
        if(costs > 0.6)
            objects(a, i) = false;
        else
            objects(a, i) = assignment(a);
        end
        
    end
          
 end
 

  assignin('base','objects', objects);
  
  for a = 1:size(objects, 1)
    moving_objects(a, 4) = objects(a, 4);
    x = moving_objects(a,4);
      for b = 5:size(objects, 2);
          x = objects(x, b - 1)
          if(x == 0)              
            continue;
          end
          moving_objects(a, b) = objects(x, b);
          
      end
      
  end
     
   assignin('base','moving_objects', moving_objects);  


end

