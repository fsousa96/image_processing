function [objects] = track3D_part1(imgseq1, cam_params)

R = cam_params.R;
T = cam_params.T;
load('CalibrationData.mat');

%% Perform background subtraction and apply the camera-to-world tranformation and the camera intrinsic parameters
nr_frames = size(imgseq1,2);
for i = 1:nr_frames
    
   imgs(:, :, i) = rgb2gray(imgseq1(i).rgb);
   imgsd(:, :, i) = double(imgseq1(i).depth)/1000; %em metros
    
end
bg_depth = median(imgsd,3);
bg_gray = median(imgs,3);
% figure(10);
% subplot(211);imagesc(bg_depth);
% subplot(212);imagesc(bg_gray);


% figure(1);clf;
% figure(2);clf;
 for i=1:nr_frames
     imdiff = imgsd(:, :, i) > 0 & imgsd(:, :, i) < 5 & abs(imgsd(:, :, i) - bg_depth)>.20;
     imgdiffiltered(:, :, i) = imopen(imdiff,strel('disk',5));
     [Gmag, Gdir] = imgradient(imgsd(:, :, i));
     
     for a = 1:size(Gmag,1)
        for b = 1:size(Gmag,2)
           if(Gmag(a, b) > 0.5)
               imgdiffiltered(a, b, i) = 0;
           end
        end   
     end
     
     %figure(1);
    % imagesc([imdiff imgdiffiltered]);
     %title('Difference image and morph filtered');
     %colormap(gray);
     %figure(2);
     %imagesc([imgseq1.depth(:,:,i) bg_depth]);
     %title('Depth image i and background image');
     %figure(3);
     connected_components(:, :, i) = bwlabel(imgdiffiltered(:, :, i));
     s = regionprops(connected_components(:, :, i), 'centroid');
     a = regionprops(connected_components(:, :, i), 'area');
     b = regionprops(connected_components(:, :, i), 'BoundingBox');
      
     figure(9);
     imagesc(bwlabel(imgdiffiltered(:, :, i)));
     title('Connected components');
     for k = 1:size(a,1)
        if(a(k).Area < 300)
           %[values, arg] = min(a(k).Area);
           %x = find(connected_components(:,:,i)  
            [x,y] = find(connected_components(:,:,i) == k);
            connected_components(x,y,i) = 0;
        end
        %connected_components(:, :, i).area = a(k).Area;      
     end
     %connected_components(:,:,i) = bwlabel(connected_components(:,:,i));
     s = regionprops(connected_components(:, :, i), 'centroid');
     a = regionprops(connected_components(:, :, i), 'area');
     b = regionprops(connected_components(:, :, i), 'BoundingBox');
%      figure(1);
%      imagesc(connected_components(:,:,i));
%      title(num2str(i));
     
     for k=1:size(s,1) % for every region 
       centroid = s(k).Centroid;
       area = a(k).Area;

     end
     
     centroids = [s.Centroid];
     areas = [a.Area];
     centroidX = centroids(1:2:end-1);
     centroidY = centroids(2:2:end);
     measurements(i).areas = areas';
     measurements(i).centroids = [centroidX' centroidY'];
     %measurements(i).ids = id;
     measurements(i).boxes = b;

     for k=1:size(s,1)
         text(centroidX(k), centroidY(k), num2str(k), 'FontSize', 14, 'FontWeight', 'Bold'); 
         measurements(i).centroids(k,3) = k;
     end
     %pause;
     
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
     
     

     



     
     %if(i == 5 || i == 4)

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
             depths(a, b, i) = imgsd(a, b, i); 
%            abs(measurements(i).areas(a) - measurements(i-1).areas(b))
               if(measurements(i).areas(a) > 400 && measurements(i-1).areas(b) > 400)
                areas(a, b, i) = abs(measurements(i).areas(a)/measurements(i-1).areas(b));
               else
                   areas(a, b, i) = NaN;
               end              
            end
            %[value, arg] = min(distances(a, :, i));
            %id(a) = arg;%returns the index of the closest centroid of the previous frame           
        end

 end
 
 for i=1:size(distances, 3)
     for a=1:size(distances, 2)
         for b=1:size(distances, 1)            
              if(areas(a, b, i) == 0)
                  areas(a, b, i) = NaN;
              end
              if(imgsd(a, b, i) > 0 && imgsd(a, b, i) < 5)
                  depths(a, b, i) = NaN;
              end
              distances(a, b, i) = distances(a, b, i) / 50;
              depths(a, b, i) = depths(a, b, i) / 25;
              cost_function(a, b, i) = abs(areas(a, b, i) - 1)*0.2 + abs(distances(a, b, i) - 1)*0.2 + abs(depths(a, b, i) - 1) *0.6; 
         end
     end
      %cost_function(:,:,i) = cost_function(:,:,i).';
 end

 assignin('base', 'distances', distances);
 assignin('base', 'areas', areas);
 assignin('base','cost_function', cost_function);
 assignin('base','connected_components', connected_components);

 
 %% check for movement
 
 for i=2:nr_frames

     [assignment, cost] = HungarianMethod(cost_function(:, :, i));
    % index = assignment(index);
    
    for a=1:length(assignment)
        if(assignment(a)~=0)
            costs = cost_function(a,assignment(a),i);
            objetos(a, i) = assignment(a);

        end
        
    end
          
 end
 

  assignin('base','objetos', objetos);
  moving_objects = zeros(size(objetos, 1),size(objetos, 2)); 
  linhas = zeros(size(moving_objects,1),1);
  for b = 1:size(objetos, 2) % colunas
      for a = 1:size(objetos, 1) % linhas
              if(objetos(a, b) ~= 0) % if an object is found
                  while_finished = false;
                  x = objetos(a, b);
                  %I want to find if, in the matrix objects, the line of
                  %the current object is an entry from the last column
                  %the line is given by find(objects(:, b) == x);
                  if(find(objetos(:, b - 1) == a)) %check if there is a previous object
                      z = b - 1; %column where that previous object is
                     while not(while_finished) % while we haven't found the "first previous"                    
                          for j = 1:size(objetos,1) %go through every line of the matrix
                             if(z == b - 1) %for finding the "most recent previous"
                                 if(objetos(j, z) == a) %if there is an object that has the value of line of the next object
                                     previous_object = objetos(j, z);
                                     break;
                                 end
                             else
                                 %previous_object
                                 find(objetos(:, z + 1) == previous_object);
                                 linez = find(objetos(:, z + 1) == previous_object);
                                 if(moving_objects(j, z) == linez)
                                     previous_object = moving_objects(j, z);
                                     break;
                                 else
                                     while_finished = true;
                                     break;
                                 end
                             
                                 
                             end
                          end
%                           if( not(isempty(find(linhas(:) == previous_object))) && z == frames(find(linhas(:) == previous_object)))
%                               previous_object;
%                               linhas(previous_object);
%                               moving_objects(find(linhas(:,1) == previous_object), b) = x;
%                               break;
%                           end
                         % if(previous
                          z = z - 1;
                          if(z == 0)    
                              while_finished = true;
                              break;

                          end
                     end
                     if(while_finished)
                          previous_object;
                          linhas(previous_object);
                          moving_objects(find(linhas(:,1) == previous_object), b) = x;
                     end

                      %object_line = objects(:, b - 1) == find(objects(:, b) == x);
                      for j=1:size(objetos, 1)                          
                          if(objetos(j, b - 1) == a)
                              y = objetos(j, b - 1);
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
                       x = objetos(a, b);
                       if(objetos(x, b + 1) ~= 0)
                           moving_objects(lineeee, b) = objetos(a, b);
                           linhas(lineeee) = objetos(a, b);
                           frames(lineeee) = b;
                       end
                  end

              end

         
      end
                      
  end
  
   for a = 1:size(moving_objects,1)
       j = 1;
       for b = 1:size(moving_objects,2)
           if(moving_objects(a, b) ~= 0)
               objects(a).frames_tracked(j) = b;
               j = j + 1;
           end

       end
   end
  
   %Remove lines with zeros
    for a = 1:size(moving_objects, 2)
       for b = size(moving_objects, 1):-1:1
          if(~any(moving_objects(b,:)))
             moving_objects(b,:) = [];
          end
       end
    end
   
   %assignin('base','frames_first_object', frames_first_object); 
   assignin('base', 'frames', frames);
   assignin('base', 'linhas', linhas);
   assignin('base','moving_objects', moving_objects);
   

   
   for i = 2:nr_frames
       
       for a = 1:size(moving_objects,1)
           
           if (moving_objects(a, i) ~= 0)
               
%                z = uint16(imgseq1(i).depth);
%                xyz1=get_xyz_asus(z(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
%                l = uint8(imgseq1(i).rgb);
%                rgbd1 = get_rgbd(xyz1, l, R_d_to_rgb, T_d_to_rgb, cam_params.Krgb);
%                figure(99);imagesc(rgbd1);
%                pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
%                figure(100); showPointCloud(pc1);
%                view(0, -85);
%                zoom(2);
               
               
            %figure(i);
            imagesc(connected_components(:,:,i));
            rectangle('Position', measurements(i).boxes(moving_objects(a, i)).BoundingBox, 'EdgeColor', 'cyan');

            %left = round( measurements(i).boxes(moving_objects(a, i)).BoundingBox(1));
            %bottom = round(measurements(i).boxes(moving_objects(a, i)).BoundingBox(2));
            %width = round(measurements(i).boxes(moving_objects(a, i)).BoundingBox(3));
            %height = round(measurements(i).boxes(moving_objects(a, i)).BoundingBox(4));
            
%             bottom_left_x = left;
%             bottom_left_y = bottom;
%             bottom_right_x = left + width;
%             bottom_right_y = bottom;
%             up_left_x = left;
%             up_left_y = bottom + height;
%             up_right_x = left + width;
%             up_right_y = bottom + height;
%             
%             K_inv = inv(cam_params.Krgb);
%             
%             x_bottom_left_m = K_inv(1,1)*bottom_left_x + K_inv(1,3);
%             y_bottom_left_m = K_inv(2,2)*bottom_left_y + K_inv(2,3);
%             
%             x_bottom_right_m = K_inv(1,1)*bottom_right_x + K_inv(1,3);
%             y_bottom_right_m = K_inv(2,2)*bottom_right_y + K_inv(2,3);
%             
%             x_up_left_m = K_inv(1,1)*up_left_x + K_inv(1,3);
%             y_up_left_m = K_inv(2,2)*up_left_y + K_inv(2,3);
%             
%             x_up_right_m = K_inv(1,1)*up_right_x + K_inv(1,3);
%             y_up_right_m = K_inv(2,2)*up_right_y + K_inv(2,3);
            
            %[x, y] = inv(cam_params.Krgb)*[bottom_left_x; bottom_left_y; 1]
%                 if(bottom + height > 480) 
%                     height = height - 1;
%                 end
%                 if(left + width > 640) 
%                     width = width - 1;
%                 end
%                 first = true;
%                 
%                 for j = bottom:bottom + height
%                     for k = left:left + width
%                         if(imgsd(j, k, i) > 0)
%                             if(connected_components(j ,k ,i) == moving_objects(a, i))
%                                 depth = imgsd(j, k, i);
%                                 if(depth < 5)
%                                     if(first)
%                                         min_depth = depth;
%                                         max_depth = depth;
%                                         first = false;
%                                     end
%                                     if (depth < min_depth)
%                                         min_depth = depth;
%                                     end
%                                     if (depth > max_depth)
%                                         max_depth = depth;
%                                     end
%                                 end
%                             end
%                         end
%                     end
%                 end
                %os pontos estão a ser guardados de baixo para cima
                %começando no canto inferior esquerdo (visto de cima) e
                %percorre-se no sentido dos ponteiros do relógio
                
                
                
%                 if(find(objects(a).frames_tracked == i))
%                  objects(a).X(i, :) = [x_bottom_left_m x_bottom_left_m x_bottom_right_m x_bottom_right_m x_bottom_left_m x_bottom_left_m x_bottom_right_m x_bottom_right_m];
%                  objects(a).Y(i, :) = [y_bottom_left_m y_bottom_left_m y_bottom_left_m y_bottom_left_m y_up_left_m y_up_left_m y_up_left_m y_up_left_m];
%                  objects(a).Z(i, :) = [min_depth max_depth max_depth min_depth min_depth max_depth max_depth min_depth];
%                 end
%                  measurements(i).depths(a, :) = [min_depth max_depth];
                          
           end

       end
   %pause
       
   end
   

%Remove lines with zeros
%     for a = 1:size(objects, 2)
%        for b = size(objects(a).X, 1):-1:1
%           if(~any(objects(a).X(b,:)))
%              objects(a).X(b,:) = [];
%              objects(a).Y(b,:) = [];
%              objects(a).Z(b,:) = [];
%           end
%        end
%     end

 assignin('base','measurements', measurements);
end

