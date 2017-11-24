clear all

nodeM = [ 0      0.063  0.093 -0.666667  0.333333  0.666667; 
          0.165  0.063  0.093  0.666667  0.666667  0.333333; 
          0.165  0      0.093  0.333333 -0.666667  0.666667;
          0      0      0.093 -0.57735 -0.57735   0.57735  ;
          0      0.063  0     -0.333333  0.666667 -0.666667;
          0.165  0.063  0      0.57735   0.57735  -0.57735 ;
          0.165  0      0      0.666667 -0.333333 -0.666667; 
          0      0      0     -0.666667 -0.666667 -0.333333];
      
faceM = [0 1 4; 
         4 1 5; 
         1 6 5; 
         1 2 6; 
         0 2 1; 
         0 3 2; 
         0 7 3; 
         0 4 7; 
         4 5 6; 
         4 6 7; 
         2 7 6; 
         2 3 7 ];

     
 


     
ptCloud = pcread('data/model/teabox.ply');
pcshow(ptCloud);

testPictureA = imread('data/images/init_texture/DSC_9743.jpg');
testPictureAGray = rgb2gray(testPictureA);
point1 = detectMinEigenFeatures(testPictureAGray,'ROI',[1344,1135,10,10]);
point2 = detectMinEigenFeatures(testPictureAGray,'ROI', [2307,1111,8,8]);
point3 = detectMinEigenFeatures(testPictureAGray,'ROI',[1373,1016,10,10]);
point4 = detectMinEigenFeatures(testPictureAGray,'ROI', [2235,1001,10,10]);
point5 = detectMinEigenFeatures(testPictureAGray,'ROI',[1368,1609,10,10]);
point6 = detectMinEigenFeatures(testPictureAGray,'ROI', [2273,1584,10,10]);
image(testPictureA);
hold on;
plot(point1.selectStrongest(50));
plot(point2.selectStrongest(50));
plot(point3.selectStrongest(50));
plot(point4.selectStrongest(50));
plot(point5.selectStrongest(50));
plot(point6.selectStrongest(50));
hold off

%%Camera constants
imageSize = size(testPictureA);
focalLength= [2960.37845 2960.37845];
principalPoint = [1841.68855 1841.68855];

CameraPointsM = [point1.Location;point2.Location;point3.Location;point4.Location;point5.Location;point6.Location];



