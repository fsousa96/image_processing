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
     connected_components(:,:,i) = bwlabel(imgdiffiltered(:,:,i));
     s = regionprops(connected_components(:,:,i), 'centroid');
     a = regionprops(connected_components(:,:,i), 'area');
     b = regionprops(connected_components(:,:,i), 'BoundingBox');
      
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
         %bounding_box(k) = b(k).BoundingBox;
     end
     
     centroids = [s.Centroid];
     areas = [a.Area];

     centroidX = centroids(1:2:end-1);
     centroidY = centroids(2:2:end);
     measurements(i).areas = areas';
     measurements(i).centroids = [centroidX' centroidY'];
     measurements(i).ids = id;
     measurements(i).boxes = b;
     
     %if(i == 5 || i == 4)
        for k=1:size(s,1)
            text(centroidX(k), centroidY(k), num2str(k), 'FontSize', 14, 'FontWeight', 'Bold'); 
            measurements(i).centroids(k,3) = k;
        end
        %pause;
     %end
     %hold on
     %plot(centroids(:,1), centroids(:, 2), 'b*');
     %hold off
     %title('Connected components');
     %pause;
     
 end
 
 for i=2:nr_frames
      %  current = imgdiffiltered(i);
       % previous = imgdiffiltered(i-1);
        %size(measurements(6).centroids,1);
        for a=1:size(measurements(i).centroids,1)
            for b=1:size(measurements(i-1).centroids,1)
             X = [measurements(i).centroids(a, 1),measurements(i).centroids(a, 2); measurements(i-1).centroids(b, 1), measurements(i-1).centroids(b, 2)];
             distances(a, b, i) = pdist(X, 'euclidean');
%             abs(measurements(i).areas(a) - measurements(i-1).areas(b))
               if(measurements(i).areas(a) > 400 && measurements(i-1).areas(b) > 400)
                areas(a, b, i) = abs(measurements(i).areas(a)/measurements(i-1).areas(b));
               else
                   areas(a, b, i) = NaN;
               end
            end
            [value, arg] = min(distances(a, :, i));
            id(a) = arg;%returns the index of the closest centroid of the previous frame
            measurements(i).centroids(a,3) = id(a);
            
        end
               
 end
 for i=1:size(distances, 3)
     for a=1:size(distances, 2)
         for b=1:size(distances, 1)            
              if(areas(a, b, i) == 0)
                  areas(a, b, i) = NaN;
              end
              
              distances(a, b, i) = distances(a, b, i) / 50;
              cost_function(a, b, i) = abs(areas(a, b, i) - 1)*0.75 + abs(distances(a, b, i) - 1)*0.25; 
         end

     end
      cost_function(:,:,i) = cost_function(:,:,i).';
 end

 assignin('base', 'distances', distances);
 assignin('base', 'areas', areas);
 assignin('base', 'id', id);
 assignin('base','cost_function', cost_function);
 assignin('base','connected_components', connected_components);

 
 %% check for movement
 
 for i=2:nr_frames

     [assignment, cost] = HungarianMethod(cost_function(:, :, i));
    % index = assignment(index);
    
    for a=1:length(assignment)
        if(assignment(a)~=0)
            costs = cost_function(a,assignment(a),i);
            if(costs > 3)
                objects(a, i) = false;
            else
                objects(a, i) = assignment(a);
            end
        end
        
    end
          
 end
 

  assignin('base','objects', objects);
  moving_objects = zeros(size(objects, 1),size(objects, 2)); 
  linhas = zeros(size(moving_objects,1),1);
  for b = 1:size(objects, 2) % colunas
      for a = 1:size(objects, 1) % linhas
              if(objects(a, b) ~= 0) % if an object is found
                  while_finished = false;
                  b;
                  x = objects(a, b);
                  %objects(:, b - 1)';
                  %I want to find if, in the matrix objects, the line of
                  %the current object is an entry from the last column
                  %the line is given by find(objects(:, b) == x);
                  if(find(objects(:, b - 1) == a)) %check if there is a previous object
                      z = b - 1; %column where that previous object is
                     while not(while_finished) % while we haven't found the "first previous"                    
                          for j = 1:size(objects,1) %go through every line of the matrix
                             if(z == b - 1) %for finding the "most recent previous"
                                 if(objects(j, z) == a) %if there is an object that has the value of line of the next object
                                     previous_object = objects(j, z);
                                     break;
                                 end
                             else
                                %previous_object
                                 find(objects(:, z + 1) == previous_object);
                                 linez = find(objects(:, z + 1) == previous_object);
                                 if(moving_objects(j, z) == linez)
                                     previous_object = moving_objects(j, z);
                                     break;
                                 else
                                     while_finished = true;
                                     break;
                                 end
                             
                                 
                             end
                          end
                          z;
%                           if( not(isempty(find(linhas(:) == previous_object))) && z == frames(find(linhas(:) == previous_object)))
%                               previous_object;
%                               linhas(previous_object);
%                               moving_objects(find(linhas(:,1) == previous_object), b) = x;
%                               break;
%                           end
                         % if(previous
                          z = z - 1;
                          if(z == 0)                             
                              break;
                              while_finished = true;
                          end
                     end
                          %só quero que corra isto quando chegar ao "first
                          %previous"
                          %previous_object
                     if(while_finished)
                          previous_object;
                          linhas(previous_object);
                          moving_objects(find(linhas(:,1) == previous_object), b) = x;
                     end

                      %object_line = objects(:, b - 1) == find(objects(:, b) == x);
                      for j=1:size(objects, 1)                          
                          if(objects(j, b - 1) == a)
                              y = objects(j, b - 1);
                              %moving_objects(a, b) = objects(a,b);
                          end 
                      end
                  else
                      for y = 1:size(moving_objects,1)
                         if(moving_objects(y,:) == 0)
                             lineeee = y;
                             break;
                         end
                      end
                       %line = find(moving_objects() == 0,1);
                       x = objects(a, b);
                       if(objects(x, b + 1) ~= 0)
                           moving_objects(lineeee, b) = objects(a, b);
                           linhas(lineeee) = objects(a, b);
                           frames(lineeee) = b;
                       end
                  end

              end

         
      end
                      
  end
   %assignin('base','frames_first_object', frames_first_object); 
   assignin('base', 'frames', frames);
   assignin('base', 'linhas', linhas);
   assignin('base','moving_objects', moving_objects);  

   
   for i = 2:nr_frames
           figure(1);
           imagesc(bwlabel(imgdiffiltered(:,:,i)));
       for a = 1:size(moving_objects,1)       
           if moving_objects(a, i) ~= 0
            rectangle('Position', measurements(i).boxes(moving_objects(a, i)).BoundingBox, 'EdgeColor', 'cyan');
            
            left =round( measurements(i).boxes(moving_objects(a, i)).BoundingBox(1));
            top = round(measurements(i).boxes(moving_objects(a, i)).BoundingBox(2));
            width = round(measurements(i).boxes(moving_objects(a, i)).BoundingBox(3));
            height = round(measurements(i).boxes(moving_objects(a, i)).BoundingBox(4));
            if(top + height > 480) 
                height = height - 1;
            end
            if(left + width > 640) 
                width = width - 1;
            end
            first = true;
            for j = top:top + height
                for k = left:left + width
                    if(imgseq1.depth(j, k, i) > 0)
                        depth = imgseq1.depth(j, k, i);
                        
                        if(first)
                            min_depth = depth;
                            max_depth = depth;
                            first = false;
                        end
                        if (depth < min_depth)
                            min_depth = depth;
                        end
                        if (depth > max_depth)
                            max_depth = depth;
                        end
                    end
                end
            end
            %guardar
            measurements(i).depths(a, :) = [min_depth max_depth];
            
           end
       end
%pause;
       
   end
   
 assignin('base','measurements', measurements);

end

