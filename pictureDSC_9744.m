clear all;

%%Geting Corner points
testPictureA = imread('data/images/init_texture/DSC_9744.jpg');
testPictureAGray = rgb2gray(testPictureA);
point1 = detectMinEigenFeatures(testPictureAGray,'ROI',[1401,984,8,8]);
point2 = detectMinEigenFeatures(testPictureAGray,'ROI', [1922,1231,8,8]);
% point3 = detectMinEigenFeatures(testPictureAGray,'ROI', [2235,1001,10,10]);
% point4 = detectMinEigenFeatures(testPictureAGray,'ROI',[1373,1016,10,10]);
% point5 = detectMinEigenFeatures(testPictureAGray,'ROI',[1368,1609,10,10]);
% point6 = detectMinEigenFeatures(testPictureAGray,'ROI', [2273,1584,10,10]);
image(testPictureA);
hold on;
plot(point1.selectStrongest(50));
plot(point2.selectStrongest(50));
% plot(point3.selectStrongest(50));
% plot(point4.selectStrongest(50));
% plot(point5.selectStrongest(50));
% plot(point6.selectStrongest(50));
hold off
% imagePointsM = [point1.Location;point2.Location;point3.Location;point4.Location;point5.Location;point6.Location];
% %imagePointsM = [point5.Location;point6.Location;point4.Location;point3.Location;point1.Location;point2.Location];
% 
% %%Camera constants
[y,x,z] = size(testPictureA);
focalLength= [2960.37845 2960.37845];
principalPoint = [1841.68855 1841.68855];
intrinsics =  cameraIntrinsics(focalLength,principalPoint,[y x]);
% 
% %% Estimating camera pose
% [worldOrientation,worldLocation] = estimateWorldCameraPose(imagePointsM,ptCloud.Location(3:8,:),intrinsics);
% 
% pcshow(prCloud.Location,'VerticalAxis','Y','VerticalAxisDir','down', ...
%      'MarkerSize',30);
%  hold on
%  plotCamera('Size',10,'Orientation',worldOrientation,'Location',...
%      worldLocation);
%  hold off