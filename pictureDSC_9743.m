clear all

%%3d model data
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
%hold on
pcshow(ptCloud);

%%Geting Corner points
testPictureA = imread('data/images/init_texture/DSC_9743.jpg');
testPictureAGray = rgb2gray(testPictureA);
point1 = detectMinEigenFeatures(testPictureAGray,'ROI',[1344,1135,10,10]);
point2 = detectMinEigenFeatures(testPictureAGray,'ROI', [2307,1111,8,8]);
point3 = detectMinEigenFeatures(testPictureAGray,'ROI', [2235,1001,10,10]);
point4 = detectMinEigenFeatures(testPictureAGray,'ROI',[1373,1016,10,10]);
point5 = detectMinEigenFeatures(testPictureAGray,'ROI',[1368,1609,10,10]);
point6 = detectMinEigenFeatures(testPictureAGray,'ROI', [2273,1584,10,10]);
% image(testPictureA);
% hold on;
% plot(point1.selectStrongest(50));
% plot(point2.selectStrongest(50));
% plot(point3.selectStrongest(50));
% plot(point4.selectStrongest(50));
% plot(point5.selectStrongest(50));
% plot(point6.selectStrongest(50));
% hold off
imagePointsM = [point1.Location;point2.Location;point3.Location;point4.Location;point5.Location;point6.Location];
%imagePointsM = [point5.Location;point6.Location;point4.Location;point3.Location;point1.Location;point2.Location];

%%Camera constants
[y,x,z] = size(testPictureA);
focalLength= [2960.37845 2960.37845];
principalPoint = [1841.68855  1235.23369];
intrinsicMatrix = [2960.37845     0         0;
                       0      2960.37845    0;
                   1841.68855 1235.23369    1];

cameraParameters =  cameraParameters('IntrinsicMatrix',intrinsicMatrix);

%% Estimating camera pose
[worldOrientation,worldLocation, inlierIdx] = estimateWorldCameraPose(imagePointsM,ptCloud.Location(1:6,:),cameraParameters,'MaxReprojectionError',18);

pcshow(ptCloud.Location,'VerticalAxis','Y','VerticalAxisDir','down', ...
     'MarkerSize',30);
 hold on
 plotCamera('Size',0.05,'Orientation',worldOrientation,'Location',...
     worldLocation);
 hold off
 
%  %%Finding sift
%  testPictureASingel = im2single(testPictureAGray);
%  fc1 = [1831 1829;1312 1312 ;1 1;0 0];
% %  fc2 = [1829;1312;1;0];
% %  fc3 = [1831;1310;1;0];
% %  fc4 = [1829;1310;1;0];
% %  fc5 = [1833;1312;1;0];
% %  fc6 = [1831;1315;1;0];
%  [F,D] = vl_sift( testPictureASingel);%,'frames',fc1)%,'frames',fc2,'frames',fc3,'frames',fc4,'frames',fc5,'frames',fc6);
%  image(testPictureA);
%  hold on;
%  vl_plotsiftdescriptor(D,F);
%  hold off;
%  
%  relevantF = [];
%  relevantD = [];
% 
%  [rows, columns] = size(F);
%  for column = 1:columns
%      if (F(1,column) >= 1377) && (F(1,column) <= 2236) && (F(2,column) >= 1016) && (F(2,column) <= 1611)
%          relevantF = [relevantF F(:,column)];
%          relevantD = [relevantD D(:,column)];
%      end
%  end
%  image(testPictureA);
%  hold on;
%  vl_plotsiftdescriptor(relevantD,relevantF);
%  hold off;
%  
%P = cameraMatrix(cameraParameters,worldOrientation,worldLocation);
Cintrinsic = [ 2960.37845     0        0;
                 0        2960.37845   0;
               1841.68855 1235.23369   1]; 
RT = [worldOrientation worldLocation'];
P = Cintrinsic*RT;
% A = eye(4);
% b = [0.165;0.063;0.093;1];
% Aeq = [0 0 0 1];
% beq= 1;
% d = [1.3458335e+03;1.1372113e+03;1];
% 
% % M = lsqlin(P',d,A,b,Aeq,beq);
M = [0.16500001; 0.063000001;0.093000002;1];
m = P*M;




