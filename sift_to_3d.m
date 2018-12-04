
function xyz_sift = sift_to_3d(img1, img2, depth1, depth2, K)
img1 = single(rgb2gray(uint8(img1)));
img2 = single(rgb2gray(uint8(img2)));

Kx = K(1,1);
Cx = K(1,3);
Ky = K(2,2);
Cy = K(2,3);

x = repmat((1:480)', 640, 1);
y = repmat((1:640), 480, 1);
y = y(:);

interp_depth1 = scatteredInterpolant(x, y, depth1(:));
interp_depth2 = scatteredInterpolant(x, y, depth2(:));

[frames1, descriptor1] = vl_sift(img1);
[frames2, descriptor2] = vl_sift(img2);
[matches, scores] = vl_ubcmatch(descriptor1, descriptor2);

xyz_sift.cam1 = zeros(3, length(scores));
xyz_sift.cam2 = zeros(3, length(scores));
for i = 1:length(scores)
    [~, idx] = min(scores);
    scores(idx) = []; % remove for the next iteration the last smallest value
    d = matches(:, idx);
    p1 = frames1(:, d(1));
    p2 = frames2(:, d(2));
    z1 = interp_depth1(p1(2), p1(1))*0.001;
    z2 = interp_depth2(p2(2), p2(1))*0.001;
    if z1 ~= 0 && z2 ~= 0
        xyz_sift.cam1(1, i) = (p1(1) - Cx)*z1/Kx;
        xyz_sift.cam1(2, i) = (p1(2) - Cy)*z1/Ky;
        xyz_sift.cam1(3, i) = z1;
        xyz_sift.cam2(1, i) = (p2(1) - Cx)*z2/Kx;
        xyz_sift.cam2(2, i) = (p2(2) - Cy)*z2/Ky;
        xyz_sift.cam2(3, i) = z2;
    end
    
    %figure
    %image(uint8(img1))
    %hold on
    %scatter(p1(1), p1(2), 100, 0.5, 'filled')
    %hold off
    %figure
    %image(uint8(img2))
    %hold on
    %scatter(p2(1), p2(2), 100, 0.5, 'filled')
    %hold off
    %pause
    
end