function [worldOrientation, worldLocation] = RansacFunction (relevantD, relevantF3D,intrinsicMatrix, imagePath)
%Load the actual image
image = imread(imagePath);
imageGray = rgb2gray(image);
%Find the sift points
testPictureASingel = im2single(imageGray);
 [F,D] = vl_sift( testPictureASingel);
 
index_pairs = matchFeatures(D',relevantD');
matchedPts2D  = F(1:2,index_pairs(:,1))';
matchedPts3D = relevantF3D(index_pairs(:,2),:);
nrMatchingPoints = size(matchedPts2D);
rng('shuffle');
bestInliers = 0;
bestWorldOrientation = [];
bestworldLocation = [];
for i = 1:500
    inliers = 0;
    try 
        randomPoints = randi([1,nrMatchingPoints(1)],4,1);
        imagePoints = [matchedPts2D(randomPoints(1),:);matchedPts2D(randomPoints(2),:);matchedPts2D(randomPoints(3),:);matchedPts2D(randomPoints(4),:)];
        treeDPoints = [matchedPts3D(randomPoints(1),:);matchedPts3D(randomPoints(2),:);matchedPts3D(randomPoints(3),:);matchedPts3D(randomPoints(4),:)];
        [worldOrientation, worldLocation] = estimateWorldCameraPose(imagePoints,treeDPoints,cameraParameters,'MaxReprojectionError',100);
  
    catch 
        continue
    end
    R = worldOrientation';
    t = -worldLocation*worldOrientation';
    RT = [R; t];
    for j = 1:nrMatchingPoints(1)
        m = [matchedPts3D(j,:) 1]*RT*intrinsicMatrix;
        d = pdist([matchedPts2D(i,:);m(1)/m(3) m(2)/m(3)],'euclidean');
        if d < 1000
            inliers = inliers + 1;
        end
    end 
    if inliers > bestInliers
        bestInliers = inliers;
        bestWorldOrientation = worldOrientation;
        bestworldLocation = worldLocation;
    end
    
end

worldOrientation = bestWorldOrientation;
worldLocation = bestworldLocation;