clear all
%% Get world points in ptCloud
% ptCloud = pcread('data/model/teabox.ply');
% % %hold on
% % pcshow(ptCloud);
%% Get real wordl points
[vertex,face] =  read_ply('data/model/teabox.ply');
singleVertex = single(vertex);


%% Geting Corner points
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

%% Camera constants
[y,x,z] = size(testPictureA);
focalLength= [2960.37845 2960.37845];
principalPoint = [1841.68855  1235.23369];
intrinsicMatrix = [2960.37845     0         0;
                       0      2960.37845    0;
                   1841.68855 1235.23369    1];

cameraParameters =  cameraParameters('IntrinsicMatrix',intrinsicMatrix);

%% Estimating camera pose
[worldOrientation,worldLocation, inlierIdx] = estimateWorldCameraPose(imagePointsM,singleVertex(1:6,:),cameraParameters,'MaxReprojectionError',18);

pcshow(singleVertex,'VerticalAxis','Y','VerticalAxisDir','down', ...
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
%% Finding rotation matrix and translation
R = worldOrientation';
t = -worldLocation*worldOrientation';
RT = [R; t];
M = [0. 0.063 0.093 1];
m = M*RT*cameraParameters.IntrinsicMatrix;
% %% Use minesquere to find the coresponding 3D point to a 2D point. 
% % A = eye(4);
% % b = [0.165;0.063;0.093;1];
% A = [];
% b = [];
% LB = [0; 0; 0; 1];
% UB = [0.165;0.063;0.093;1];
% Aeq = [0 0 0 1];
% beq= 1;
% d = [1.3458335e+03;1.1372113e+03;1];
% P = cameraParameters.IntrinsicMatrix'*RT';
% M_est = lsqlin(P,d,A,b,Aeq,beq,LB,UB);
% m_est = P*M';
%% Corelating 2D point to 3D
m_test = [1.3458335e+03;1.1372113e+03;1;1];
%m_test = [1.3458335e+03;1.1372113e+03;1];
invertebleP = [cameraParameters.IntrinsicMatrix' zeros(3,1);zeros(1,3) 1]*[R' t'; zeros(1,3) 1]*[cameraParameters.IntrinsicMatrix' zeros(3,1);zeros(1,3) 1];
invP = inv(invertebleP);
inverteblePReduced = invertebleP(1:3,:);
%dir = inverteblePReduced*m_test;
% invRTFull = inv([R' t';zeros(1,3) 1]);
% invRTReduced = invRTFull(1:3,:);
% invP = invRTReduced*inv(cameraParameters.IntrinsicMatrix);
dir = invP*m_test;
vert0 = vertex(1,:)';
vert1 = vertex(2,:)';
vert2 = vertex(3,:)';
[intersect, distance, u, v, xcoor] = TriangleRayIntersection(t, dir(1:3), vert0, vert1, vert2);



