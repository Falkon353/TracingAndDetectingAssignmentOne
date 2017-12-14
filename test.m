%function r = Ransac (pictureDSC_9743,inlier, outlier )  
%Reads the original image
DSC_9743 = imread('data/images/init_texture/DSC_9744.jpg');
imshow(DSC_9743); 
title('DSC_image');
%Transforms the original image
distorted = imresize(DSC_9743,0.7);
distorted = imrotate(distorted,31);
figure; 
imshow(distorted);
title('TransformedImage');
%Detect and extraxt features from the images
DSC_9743Gray = rgb2gray(DSC_9743);
distortedGray = rgb2gray(distorted);
singelDSC_9743 = im2single(DSC_9743Gray);
singelDistorted = im2single(distortedGray);
ptsDSC_9743  = vl_sift(singelDSC_9743);
ptsDistorted = vl_sift(singelDistorted);
[featuresDSC_9743,validPtsDSC_9743] = extractFeatures(DSC_9743Gray,ptsDSC_9743(1:2,:)');
[featuresDistorted,validPtsDistorted] = extractFeatures(distortedGray,ptsDistorted(1:2,:)');
%Match the image features
index_pairs = matchFeatures(featuresDSC_9743,featuresDistorted);
matchedPtsDSC_9743  = validPtsDSC_9743(index_pairs(:,1),:);
matchedPtsDistorted = validPtsDistorted(index_pairs(:,2),:);
figure; 
showMatchedFeatures(DSC_9743,distorted,...
    matchedPtsDSC_9743,matchedPtsDistorted);
title('Matched SIFT points,including outliers');
%Exclude the outliers 
[tform,inlierPtsDistorted,inlierPtsDSC_9743] = ...
    estimateGeometricTransform(matchedPtsDistorted,matchedPtsDSC_9743,'similarity');
figure; 

showMatchedFeatures(DSC_9743,distorted,...
    inlierPtsDSC_9743,inlierPtsDistorted);
title('Matched inlier points');