p1 = rand(3,20) *5;
[R, ~,~] = svd(rand(theta));
if(det(R) == -1 ) 
    R = - R;
end
T = 10*rand(3,1);
p2 = R * p1 + T*ones(1,20);
outliers = [1 3 5 18 19];
p2(1, outliers) = -p2(1,outliers);
inliers = ranscac(p1,p2,erro1,erro2);