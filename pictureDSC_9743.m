clear all
%% Get world points in ptCloud
% ptCloud = pcread('data/model/teabox.ply');
% % %hold on
% % pcshow(ptCloud);
%% Get real wordl points
[vertex,face] =  read_ply('data/model/teabox.ply');
singleVertex = single(vertex);
centerPoint = cornerPoints([1841.68855 1235.2336]);


%% Geting Corner points
testPictureA = imread('data/images/init_texture/DSC_9743.jpg');
testPictureAGray = rgb2gray(testPictureA);
point1 = detectMinEigenFeatures(testPictureAGray,'ROI',[1344,1135,10,10]);
point2 = detectMinEigenFeatures(testPictureAGray,'ROI', [2307,1111,8,8]);
point3 = detectMinEigenFeatures(testPictureAGray,'ROI', [2235,1001,10,10]);
point4 = detectMinEigenFeatures(testPictureAGray,'ROI',[1373,1016,10,10]);
point5 = detectMinEigenFeatures(testPictureAGray,'ROI',[1368,1609,10,10]);
point6 = detectMinEigenFeatures(testPictureAGray,'ROI', [2273,1584,10,10]);
image(testPictureA);
hold on;
% plot(point1.selectStrongest(50));
% plot(point2.selectStrongest(50));
% plot(point3.selectStrongest(50));
% plot(point4.selectStrongest(50));
% plot(point5.selectStrongest(50));
% plot(point6.selectStrongest(50));
plot(centerPoint);
hold off
imagePointsM = [point1.Location;point2.Location;point3.Location;point4.Location;point5.Location;point6.Location];
mapedVertex = [singleVertex(4,:);singleVertex(3,:);singleVertex(2,:);singleVertex(1,:);singleVertex(8,:);singleVertex(7,:)];
%% Camera constants
[y,x,z] = size(testPictureA);
focalLength= [2960.37845 2960.37845];
principalPoint = [1841.68855  1235.23369];
intrinsicMatrix = [2960.37845     0         0;
                       0      2960.37845    0;
                   1841.68855 1235.23369    1];

cameraParameters =  cameraParameters('IntrinsicMatrix',intrinsicMatrix);

%% Estimating camera pose
[worldOrientation,worldLocation, inlierIdx] = estimateWorldCameraPose(imagePointsM,mapedVertex,cameraParameters,'MaxReprojectionError',3);

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
worldPoints = [0. 0.063 0.093];
m = M*RT*cameraParameters.IntrinsicMatrix;
%% Corelating 2D point to 3D
m_test = [1.3458e+03;1.1372e+03;1];
centerDirection = worldOrientation'*[0; 0; 1];
dirx = centerDirection(1)+(m_test(1)-principalPoint(1))*focalLength(1);
diry = centerDirection(2)+(m_test(2)-principalPoint(2))*focalLength(2);
dirz = centerDirection(3);
% dir = [dirx; diry; dirz];
dir = intrinsicMatrix'*m_test;
unitDir = dir/norm(dir);
DirRotated = unitDir;
vert0 = vertex(3,:)';
vert1 = vertex(4,:)';
vert2 = vertex(8,:)';
[intersect, distance, u, v, xcoor] = TriangleRayIntersection(worldLocation, centerDirection, vert0, vert1, vert2,'border', 'inclusive');


pcshow(singleVertex,[0 0 0],'VerticalAxis','Y','VerticalAxisDir','down', ...
     'MarkerSize',30);
 hold on
 plotCamera('Size',0.05,'Orientation',worldOrientation,'Location',...
     worldLocation);
 quiver3(worldLocation(1),worldLocation(2),worldLocation(3),centerDirection(1),centerDirection(2),centerDirection(3))
 quiver3(worldLocation(1),worldLocation(2),worldLocation(3),DirRotated(1),DirRotated(2),DirRotated(3))
 %quiver3(0,0,0,DirRotated(1),DirRotated(2),DirRotated(3))
 scatter3(xcoor(1),xcoor(2),xcoor(3));
 hold off

% hold on;
% vertexDim = size(mapedVertex);
% for i= 1:vertexDim(1)
%     m1 = [mapedVertex(i,:) 1]*RT*cameraParameters.IntrinsicMatrix;
%     disp("Vertex nr:");
%     disp(i);
%     m1_1 = m1(1)/m1(3)
%     m1_2 = m1(2)/m1(3)
%     for j = i:vertexDim(1)
%         disp("VErtex second nr")
%         disp(j);
%         m2 = [mapedVertex(j,:) 1]*RT*cameraParameters.IntrinsicMatrix;
%         line([m1(1)/m1(3),m2(1)/m2(3)],[m1(2)/m1(3),m2(2)/m2(3)]);
%     end
% end
% hold off; 



