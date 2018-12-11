function [objects] = track3D_part1(imgseq1, cam_params)

R = cam_params.R;
T = cam_params.T;
load('CalibrationData.mat');

% Perform background subtraction and apply the camera-to-world tranformation and the camera intrinsic parameters
nr_frames = size(imgseq1,2);
for i = 1:nr_frames
    
   imgs(:, :, i) = rgb2gray(imgseq1(i).rgb);
   imgsd(:, :, i) = double(imgseq1(i).depth)/1000; %em metros
    
end
assignin('base', 'imgsd', imgsd);
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
      


     for k = 1:size(a,1)
        if(a(k).Area < 400)
           %[values, arg] = min(a(k).Area);
           %x = find(connected_components(:,:,i)  
            [x,y] = find(connected_components(:,:,i) == k);
            connected_components(x,y,i) = 0;
        end
        %connected_components(:, :, i).area = a(k).Area;      
     end

     connected_components(:,:,i) = bwlabel(connected_components(:,:,i));
     s2 = regionprops(connected_components(:, :, i), 'centroid');
     a2 = regionprops(connected_components(:, :, i), 'area');
     b2 = regionprops(connected_components(:, :, i), 'BoundingBox');
  
%      figure(1);

    

%      for k = 1:size(s2,1) % for every region 
% %        rectangle('Position', b(k).BoundingBox, 'EdgeColor', 'cyan');
% %         left = round( b(k).BoundingBox(1));
% %         bottom = round(b(k).BoundingBox(2));
% %         width = round(b(k).BoundingBox(3));
% %         height = round(b(k).BoundingBox(4));
% %         if(bottom + height > 480) 
% %             height = height - 1;
% %         end
% %         if(left + width > 640) 
% %             width = width - 1;
% %         end
%         %lines = bottom:bottom + height;
%         %columns = left:left+ width;
%        % z = uint16(imgseq1(i).depth(lines,columns));
%       %  z2 = uint16(imgseq1(i).depth);
%                 
% %         for j = bottom:bottom + height
% %             for k = left:left + width
% %                 if(connected_components(j,k,i) ~= 3)
% %                     z(j,k) = 0;
% %                 end
% %             end
% %         end
%         %imagesc(z);
%         %figure;
%         %imagesc(z2);
%         %z2(lines,columns)
%       %  xyz=get_xyz_asus(z2(lines, columns),[size(z2,1) size(z2,2)],(1:size(z2,1)*size(z2,2))', cam_params.Kdepth,1,0);
%        % xyz1=get_xyz_asus(z2(),[100 100],(1:100*100)', cam_params.Kdepth,1,0);
% %         l = uint8(imgseq1(i).rgb([bottom:bottom+height],[left:left+width]));
% %         rgbd1 = get_rgbd(xyz1, l, R_d_to_rgb, T_d_to_rgb, cam_params.Krgb);
% %         pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[size(z,1)*size(z,2) 3]));
% %         figure(100); showPointCloud(pc1);
%                
% %         for j = bottom:bottom + height
% %             for k = left:left + width
% %             end
% %         end
%      end
     
     centroids = [s2.Centroid];
     areas = [a2.Area];
     centroidX = centroids(1:2:end-1);
     centroidY = centroids(2:2:end);
     measurements(i).areas = areas';
     measurements(i).centroids = [centroidX' centroidY'];
     %measurements(i).ids = id;
              imagesc(connected_components(:, :, i));
         title(num2str(i));
     for k=1:size(s2,1)

         text(centroidX(k), centroidY(k), num2str(k), 'FontSize', 14, 'FontWeight', 'Bold'); 
         measurements(i).centroids(k,3) = k;
         %rectangle('Position', b(k).BoundingBox, 'EdgeColor', 'cyan');
         
         left = round(b2(k).BoundingBox(1));
         bottom = round(b2(k).BoundingBox(2));
         width = round(b2(k).BoundingBox(3));
         height = round(b2(k).BoundingBox(4));
         if(bottom + height > 480) 
            height = height - 1;
         end
         if(left + width > 640) 
            width = width - 1;
         end
         lines = bottom:bottom + height;
         columns = left:left+ width;
         z = uint16(imgseq1(i).depth(lines,columns));
        % imagesc(z2);      
         for f = lines(1):lines(end)
             for g = columns(1):columns(end)
                 if(connected_components(f,g,i) ~= k)
                     z(f-lines(1) + 1, g - columns(1) + 1) = 1000000;
                 end
             end
         end

         xyz = get_xyz_asus(z(:), [size(lines,2) size(columns,2)], (1:size(lines,2)*size(columns,2))',cam_params.Kdepth,1,0);
         first_x = false;
         first_y = false;
         first_z = false;
         for x = 1:size(xyz,1)
             x;
             if(xyz(x,1) > -10 && xyz(x,2) > -10 && xyz(x,3) < 10)
                 
                 if(xyz(x,1) > -10)
                     if(not(first_x))
                        x_max = xyz(x,1);
                        x_min = xyz(x,1); 
                        first_x = true;

                     else
                        if(xyz(x,1) < x_min)
                            x_min = xyz(x,1);
                        elseif (xyz(x,1) > x_max)
                            x_max = xyz(x,1);
                        end
                     end
                 end

                 if(xyz(x,2) > -5)
                     if(not(first_y))
                        y_max = xyz(x,2);
                        y_min = xyz(x,2); 
                        first_y = true;

                     else
                        if(xyz(x,2) < y_min)
                            y_min = xyz(x,2);
                        elseif (xyz(x,2) > y_max)
                            y_max = xyz(x,2);
                        end
                     end
                 end
                 
                 if(xyz(x,3) < 10)
                     if(not(first_z))
                        z_max = xyz(x,3);
                        z_min = xyz(x,3); 
                        first_z = true;
                     else
                        if(xyz(x,3) < z_min)
                            z_min = xyz(x,3);
                        elseif (xyz(x,3) > z_max)
                            z_max = xyz(x,3);
                        end
                     end
                 end


             end              
         end
         %come�a ponto mais em baixo, mais � esquerda, mais perto. Depois
         %vai em sentido dos ponteiros do rel�gio e depois passa para cima
         measurements(i).boxes(k).X = [x_min x_min x_max x_max x_min x_min x_max x_max];
         measurements(i).boxes(k).Y = [y_min y_min y_min y_min y_max y_max y_max y_max];
         measurements(i).boxes(k).Z = [z_min z_max z_max z_min z_min z_max z_max z_min];
%          hold on;
%          %figure(2);
%          cube_plot([x_max-x_min y_max-y_min z_max-z_min],[x_min y_min z_min],.5,[1 0 0]);
%          %xyz1 = get_xyz_asus(z2(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
%          L = uint8(imgseq1(i).rgb(lines, columns,:));
%          %L1 = uint8(imgseq1(i).rgb);
%          rgbd1 = get_rgbd(xyz, L, R_d_to_rgb, T_d_to_rgb, cam_params.Krgb, size(lines,2), size(columns,2));
%          %rgbd2 = get_rgbd(xyz1, L1, R_d_to_rgb, T_d_to_rgb, cam_params.Krgb, 480, 640);
%          pc1=pointCloud(xyz,'Color',reshape(rgbd1,[size(lines,2)*size(columns,2) 3]));
% 
%          showPointCloud(pc1);
%          view(1,-88);
%          zoom(1.5);
%  
%          axis([x_min-0.2 x_max+0.2  y_min-0.2 y_max+0.2  z_min-0.2 z_max+0.2]);

     end
%        hold off;
%        pause(1);
%pause;
 end
 

 for i=2:nr_frames
      %  current = imgdiffiltered(i);
       % previous = imgdiffiltered(i-1);
        %size(measurements(6).centroids,1);
        for a=1:size(measurements(i).boxes,2)
            for b=1:size(measurements(i-1).boxes,2)
             %X = [measurements(i).centroids(a, 1),measurements(i).centroids(a, 2); measurements(i-1).centroids(b, 1), measurements(i-1).centroids(b, 2)];
             %distances(a, b, i) = pdist(X, 'euclidean');            
             %depths(a, b, i) = imgsd(a, b, i);  %aqui tenho de ver as coordenadas do centroid
%            abs(measurements(i).areas(a) - measurements(i-1).areas(b))
            distanceX(a, b, i) = norm(measurements(i).boxes(a).X - measurements(i-1).boxes(b).X);
            distanceY(a, b, i) = norm(measurements(i).boxes(a).Y - measurements(i-1).boxes(b).Y);
            distanceZ(a, b, i) = norm(measurements(i).boxes(a).Z - measurements(i-1).boxes(b).Z);
            sum = 0;
            for x = 1:8
                dist = norm([measurements(i).boxes(a).X(x) measurements(i).boxes(a).Y(x) measurements(i).boxes(a).Z(x)] ...
                    - [measurements(i-1).boxes(b).X(x) measurements(i-1).boxes(b).Y(x) measurements(i-1).boxes(b).Z(x)]);
                sum = sum + dist;
            end
            distances(a, b, i) = sum / 8;
            % if(measurements(i).areas(a) > 400 && measurements(i-1).areas(b) > 400)
            areas(a, b, i) = abs(measurements(i).areas(a)/measurements(i-1).areas(b));
               %else
                %   areas(a, b, i) = NaN;
               %end              
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
              if(distances(a, b, i) > 0.5)
                  distances(a, b, i) = NaN;
              end
              %distances(a, b, i) = distances(a, b, i) / 50;
              %depths(a, b, i) = depths(a, b, i) / 25;
              cost_function(a, b, i) = abs(areas(a, b, i) - 1)*0.1 + distances(a, b, i)*0.9 ; 
         end
     end
      cost_function(:,:,i) = cost_function(:,:,i).';
 end

 assignin('base', 'distances', distances);
 assignin('base', 'areas', areas);
% assignin('base', 'depths', depths);
 assignin('base','cost_function', cost_function);
 assignin('base','connected_components', connected_components);

 
 % check for movement
 
 for i=2:nr_frames

     [assignment, cost] = HungarianMethod(cost_function(:, :, i));
    % index = assignment(index);
    
    for a=1:length(assignment)
        if(assignment(a) ~= 0)
            costs = cost_function(a,assignment(a),i);
            objetos(a, i) = assignment(a);

        end
        
    end
          
 end
 

  assignin('base','objetos', objetos);
  moving_objects = zeros(size(objetos, 1),size(objetos, 2)); 
  linhas = zeros(size(moving_objects,1),1);
%   for b = 1:size(objetos, 2) % colunas
%       for a = 1:size(objetos, 1) % linhas
%               if(objetos(a, b) ~= 0) % if an object is found
%                   while_finished = false;
%                   flag = false;
%                   x = objetos(a, b)
%                   b
%                   %I want to find if, in the matrix objects, the line of
%                   %the current object is an entry from the last column
%                   %the line is given by find(objects(:, b) == x);
%                   if(find(objetos(:, b - 1) == a)) %check if there is a previous object
%                       z = b - 1 %column where that previous object is
%                      while (not(while_finished)) % while we haven't found the "first previous"
%                           for j = 1:size(objetos,1) %go through every line of the matrix
%                              if(z == b - 1) %for finding the "most recent previous"
%                                  if(objetos(j, z) == a) %if there is an object that has the value of line of the next object
%                                      previous_object = objetos(j, z);
%                                      break;
%                                  end
%                              else
%                                  z + 1;
%                                  previous_object
%                                  j
%                                  z
%                                  moving_objects(j, z)
%                                  find(objetos(:, z + 1) == previous_object)
%                                  linez = find(objetos(:, z + 1) == previous_object)
%                                  if(moving_objects(j, z) == linez)
%                                      previous_object = moving_objects(j, z)
%                                      break;
%                                  else
%                                      while_finished = true;
%                                      flag = true;
%                                      z
%                                      break;
%                                  end                                
%                              end
%                           end
%                           if(flag)
%                               break;
%                           end
%                           
%                           if(z - 1 ~= 0) 
%                               z = z - 1;
%                               
%                           else
%                               while_finished = true;
%                               break;
% 
%                           end
%                      end
%                      if(while_finished)
%                           x
%                           previous_object %preciso de saber qual a localiza��o deste mambo
%                           z + 1
%                           linhas(previous_object);
%                           %find(first_objects(:,) == previous_object)
%                           find(first_objects(:,z + 1) == previous_object)
%                           moving_objects(find(first_objects(:,z + 1) == previous_object), b) = x
%                          % moving_objects(find(linhas(:,1) == previous_object), b) = x
%                           %moving_objects(linhas(previous_object), b) = x
%                           %moving_objects(lineeee, b) = x
%                      end
% 
%                       %object_line = objects(:, b - 1) == find(objects(:, b) == x);
%                       for j=1:size(objetos, 1)                          
%                           if(objetos(j, b - 1) == a)
%                               y = objetos(j, b - 1);
%                               %moving_objects(a, b) = objects(a,b);
%                           end 
%                       end
%                   else
%                       for y = 1:size(moving_objects,1)
%                          if(moving_objects(y,:) == 0)
%                              lineeee = y
%                              break;
%                          end
%                       end
%                        %line = find(moving_objects() == 0,1);
%                        x = objetos(a, b);
%                        b;
%                        if(objetos(x, b + 1) ~= 0)
%                            moving_objects(lineeee, b) = objetos(a, b);
%                            linhas(lineeee) = objetos(a, b);
%                            first_objects(lineeee, b) = objetos(a, b)
%                            frames(lineeee) = b;
%                        end
%                   end
% 
%               end        
%       end
%                       
%   end
%   
  y = 0;
  for b = 1:size(objetos, 2)
     for a = 1:size(objetos ,1)
         if(objetos(a, b) ~= 0)
             a;
             b;
            test1 = find(objetos(:, b - 1) == a); %linha do objeto anterior
            if(not(isempty(test1))) 
               z = b - 1;
               previous_object = objetos(test1, z);
                 if(isempty(find(objetos(:, z - 1) == test1)) || not(any(objetos(:, z - 1))))
                    %if(not(any(objetos(:, j - 1))) | )
                      % disp('chegou ao fim');
                       linha = find(first_objects(:, z) == previous_object);
                       moving_objects(linha, b) = objetos(a, b);
%                        objects(a).X(b,:) = measurements(a).boxes(b).X;
%                        objects(a).Y(b,:) = measurements(a).boxes(b).Y;
%                        objects(a).Z(b,:) = measurements(a).boxes(b).Z;
                       %por na matriz moving_objects
                 else
                    for j = z-1:-1:2
                   
                        test = find(objetos(:, j) == test1);
                        if(not(isempty(test)))
                            previous_object = objetos(test, j) ;
                            test1 = test;
                        end
                        objetos(:, j- 1);
                        if(isempty(find(objetos(:, j - 1) == test1)) || not(any(objetos(:, j - 1))))
                            %if(not(any(objetos(:, j - 1))) | )
                           % disp('chegou ao fim');
                            linha = find(first_objects(:, j) == previous_object);
                            moving_objects(linha, b) = objetos(a, b);
%                             objects(a).X(b,:) = measurements(a).boxes(b).X;
%                             objects(a).Y(b,:) = measurements(a).boxes(b).Y;
%                             objects(a).Z(b,:) = measurements(a).boxes(b).Z;
                            %por na matriz moving_objects
                       
                            break;
                        end
                    end
                 end
            else
                %cria novo moving_object
                %disp('novo');
                %Preciso de verificar se este objeto j� apareceu
                %como � que posso obter as caracter�sticas deste objeto
                %(�rea e posi��o)
                %centroid = measurements(a).centroids(:, find(measurements(a).centroids(:,3) == objetos(a ,b)));
                y = y + 1;
                first_objects(y, b) = objetos(a, b);
                moving_objects(y, b) = objetos(a, b);
%                 objects(a).X(b,:) = measurements(a).boxes(b).X;
%                 objects(a).Y(b,:) = measurements(a).boxes(b).Y;
%                 objects(a).Z(b,:) = measurements(a).boxes(b).Z;
                
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
   
  for b = 2:nr_frames      
      for a = 1:size(moving_objects,1);
          if(moving_objects(a, b) ~= 0)
              
               if(find(objects(a).frames_tracked == b))
                 objects(a).X(b, :) = measurements(b).boxes(find(measurements(b).centroids(:,3) == moving_objects(a, b))).X;
                 objects(a).Y(b, :) = measurements(b).boxes(find(measurements(b).centroids(:,3) == moving_objects(a, b))).Y;
                 objects(a).Z(b, :) = measurements(b).boxes(find(measurements(b).centroids(:,3) == moving_objects(a, b))).Z;
               end
          end          
          
      end
  end
  

 %Remove lines with zeros
    for a = 1:size(objects, 2)
       for b = size(objects(a).X, 1):-1:1
          if(~any(objects(a).X(b,:)))
             objects(a).X(b,:) = [];
             objects(a).Y(b,:) = [];
             objects(a).Z(b,:) = [];
          end
       end
    end
   %size(objects,1) 
    for a = size(objects, 2)
       if(length(objects(a).frames_tracked) == 1)
            objects(a) = [];
       end
    end
      assignin('base','moving_objects', moving_objects);
     % assignin('base','objects', objects);
  
  
  %%
  
  
  
  
  
   
   
   
   
   
   
   
   
   
  
%    for a = 1:size(moving_objects,1)
%        j = 1;
%        for b = 1:size(moving_objects,2)
%            if(moving_objects(a, b) ~= 0)
%                objects(a).frames_tracked(j) = b;
%                j = j + 1;
%            end
% 
%        end
%    end
%   

%    
%    %assignin('base','frames_first_object', frames_first_object); 
%    assignin('base', 'frames', frames);
%    assignin('base', 'linhas', linhas);
%    assignin('base','moving_objects', moving_objects);
%    
% 
%    
%    for i = 2:nr_frames
%        
%        for a = 1:size(moving_objects,1)
%            
%            if (moving_objects(a, i) ~= 0)
%                
% %                z = uint16(imgseq1(i).depth);
% %                xyz1=get_xyz_asus(z(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
% %                l = uint8(imgseq1(i).rgb);
% %                rgbd1 = get_rgbd(xyz1, l, R_d_to_rgb, T_d_to_rgb, cam_params.Krgb);
% %                figure(99);imagesc(rgbd1);
% %                pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
% %                figure(100); showPointCloud(pc1);
% %                view(0, -85);
% %                zoom(2);
%                
%                
%             %figure(i);
%             imagesc(connected_components(:,:,i));
%             rectangle('Position', measurements(i).boxes(moving_objects(a, i)).BoundingBox, 'EdgeColor', 'cyan');
% 
%             %left = round( measurements(i).boxes(moving_objects(a, i)).BoundingBox(1));
%             %bottom = round(measurements(i).boxes(moving_objects(a, i)).BoundingBox(2));
%             %width = round(measurements(i).boxes(moving_objects(a, i)).BoundingBox(3));
%             %height = round(measurements(i).boxes(moving_objects(a, i)).BoundingBox(4));
%             
% %             bottom_left_x = left;
% %             bottom_left_y = bottom;
% %             bottom_right_x = left + width;
% %             bottom_right_y = bottom;
% %             up_left_x = left;
% %             up_left_y = bottom + height;
% %             up_right_x = left + width;
% %             up_right_y = bottom + height;
% %             
% %             K_inv = inv(cam_params.Krgb);
% %             
% %             x_bottom_left_m = K_inv(1,1)*bottom_left_x + K_inv(1,3);
% %             y_bottom_left_m = K_inv(2,2)*bottom_left_y + K_inv(2,3);
% %             
% %             x_bottom_right_m = K_inv(1,1)*bottom_right_x + K_inv(1,3);
% %             y_bottom_right_m = K_inv(2,2)*bottom_right_y + K_inv(2,3);
% %             
% %             x_up_left_m = K_inv(1,1)*up_left_x + K_inv(1,3);
% %             y_up_left_m = K_inv(2,2)*up_left_y + K_inv(2,3);
% %             
% %             x_up_right_m = K_inv(1,1)*up_right_x + K_inv(1,3);
% %             y_up_right_m = K_inv(2,2)*up_right_y + K_inv(2,3);
%             
%             %[x, y] = inv(cam_params.Krgb)*[bottom_left_x; bottom_left_y; 1]
% %                 if(bottom + height > 480) 
% %                     height = height - 1;
% %                 end
% %                 if(left + width > 640) 
% %                     width = width - 1;
% %                 end
% %                 first = true;
% %                 
% %                 for j = bottom:bottom + height
% %                     for k = left:left + width
% %                         if(imgsd(j, k, i) > 0)
% %                             if(connected_components(j ,k ,i) == moving_objects(a, i))
% %                                 depth = imgsd(j, k, i);
% %                                 if(depth < 5)
% %                                     if(first)
% %                                         min_depth = depth;
% %                                         max_depth = depth;
% %                                         first = false;
% %                                     end
% %                                     if (depth < min_depth)
% %                                         min_depth = depth;
% %                                     end
% %                                     if (depth > max_depth)
% %                                         max_depth = depth;
% %                                     end
% %                                 end
% %                             end
% %                         end
% %                     end
% %                 end
%                 %os pontos est�o a ser guardados de baixo para cima
%                 %come�ando no canto inferior esquerdo (visto de cima) e
%                 %percorre-se no sentido dos ponteiros do rel�gio
%                 
%                 
%                 
% %                 if(find(objects(a).frames_tracked == i))
% %                  objects(a).X(i, :) = [x_bottom_left_m x_bottom_left_m x_bottom_right_m x_bottom_right_m x_bottom_left_m x_bottom_left_m x_bottom_right_m x_bottom_right_m];
% %                  objects(a).Y(i, :) = [y_bottom_left_m y_bottom_left_m y_bottom_left_m y_bottom_left_m y_up_left_m y_up_left_m y_up_left_m y_up_left_m];
% %                  objects(a).Z(i, :) = [min_depth max_depth max_depth min_depth min_depth max_depth max_depth min_depth];
% %                 end
% %                  measurements(i).depths(a, :) = [min_depth max_depth];
%                           
%            end
% 
%        end
%    %pause
%        
%    end
%    
% 

% 

  assignin('base','measurements', measurements);
end

