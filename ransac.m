function bestinliers = ransac(iter, points_cam1, points_cam2, thresh_error, thresh_inlier)
nbpairs = length(points_cam1);
bestinliers.cam1 = [];
bestinliers.cam2 = [];
for i = 1:iter
    idx = randperm(nbpairs, 4); 
    sample1 = points_cam1(:, idx)';
    sample2 = points_cam2(:, idx)';
    [~, ~, transform] = procrustes(sample1, sample2, 'scaling', false, 'reflection', false);
    R = transform.T;
    T = transform.c;
    inliers.cam1 = sample1;
    inliers.cam2 = sample2;
    for j = 1:nbpairs
        error = norm(points_cam1(:, j) - R'*points_cam2(:, j) - T(1, :)');
        if error < thresh_error
            inliers.cam1 = [inliers.cam1; points_cam1(:, j)'];
            inliers.cam2 = [inliers.cam2; points_cam2(:, j)'];
        end
    end
    nbinliers = length(inliers.cam1);
    if nbinliers > length(bestinliers.cam1) %&& nbinliers > thresh_inlier*nbpairs 
        bestinliers.cam1 = inliers.cam1;
        bestinliers.cam2 = inliers.cam2;
    end
end