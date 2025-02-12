clear all;
run('vlfeat-0.9.20/toolbox/vl_setup');
%% Get real wordl points
[vertex,faces] =  read_ply('data/model/teabox.ply');
faces = faces+1;
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
% image(testPictureA);
% hold on;
% plot(point1.selectStrongest(50));
% plot(point2.selectStrongest(50));
% plot(point3.selectStrongest(50));
% plot(point4.selectStrongest(50));
% plot(point5.selectStrongest(50));
% plot(point6.selectStrongest(50));
% plot(centerPoint);
% hold off
imagePointsM = [point1.Location;point2.Location;point3.Location;point4.Location;point5.Location;point6.Location];
mapedVertex = [singleVertex(4,:);singleVertex(3,:);singleVertex(2,:);singleVertex(1,:);singleVertex(8,:);singleVertex(7,:)];
visebleFaces = [faces(5,:);faces(6,:);faces(11,:);faces(12,:)];
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
R = worldOrientation';
t = -worldLocation*worldOrientation';
RT = [R; t];

 %%Finding sift
 testPictureASingel = im2single(testPictureAGray);
 [F,D] = vl_sift( testPictureASingel);
 image(testPictureA);
 hold on;
 vl_plotsiftdescriptor(D,F);
 hold off;
 
 relevantF = [];
 relevantD = [];
 relevantF3D = [];

 [rows, columns] = size(F);
 [rowVisebleFaces, columnsVisebleFaces] = size(visebleFaces);
 for column = 1:columns
     if (F(1,column) >= 1377) && (F(1,column) <= 2236) && (F(2,column) >= 1016) && (F(2,column) <= 1611)
         for face = 1:rowVisebleFaces
             dir = inv(intrinsicMatrix')*[F(1,column);F(2,column);1];
             unitDir = dir/norm(dir);
             dirRotated = R*unitDir;
             [intersect, distance, u, v, xcoor] = TriangleRayIntersection(worldLocation, dirRotated, vertex(visebleFaces(face,1),:)', ...
                 vertex(visebleFaces(face,2),:)',vertex(visebleFaces(face,3),:)','border', 'inclusive');
             if intersect == 1
                relevantF = [relevantF F(:,column)];
                relevantD = [relevantD D(:,column)];
                relevantF3D = [relevantF3D; xcoor];
             end
         end
     end
 end
 image(testPictureA);
 hold on;
 vl_plotsiftdescriptor(relevantD,relevantF);
 hold off;

%% Corelating 2D point to 3D
% %m_test = [1.3458e+03;1.1372e+03;1];
% M = [0.020 0. 0.045 1];
% m = M*RT*cameraParameters.IntrinsicMatrix;
% m_test = M*RT*cameraParameters.IntrinsicMatrix;
% centerDirection = worldOrientation'*[0; 0; 1];
% dir = inv(intrinsicMatrix')*[m_test(1)/m_test(3);m_test(2)/m_test(3);m_test(3)/m_test(3)];
% unitDir = dir/norm(dir);
% DirRotated = R*unitDir;
% vert0 = vertex(4,:)';
% vert1 = vertex(7,:)';
% vert2 = vertex(8,:)';
% [intersect, distance, u, v, xcoor] = TriangleRayIntersection(worldLocation, DirRotated, vert0, vert1, vert2,'border', 'inclusive');

%% Plot camera and vectors from camer
% pcshow(singleVertex,[0 0 0],'VerticalAxis','Y','VerticalAxisDir','down', ...
%      'MarkerSize',30);
%  hold on
%  plotCamera('Size',0.05,'Orientation',worldOrientation,'Location',...
%      worldLocation);
%  quiver3(worldLocation(1),worldLocation(2),worldLocation(3),centerDirection(1),centerDirection(2),centerDirection(3))
%  quiver3(worldLocation(1),worldLocation(2),worldLocation(3),DirRotated(1),DirRotated(2),DirRotated(3))
%  scatter3(M(1),M(2),M(3));
%  scatter3(xcoor(1),xcoor(2),xcoor(3));
%  hold off

%% Plots eges on the image
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
R(1,1) = R(1,1);%+0.001;
R(2,2) = R(2,2);%+0.001;
mi = [1.3458e+03 1.1372e+03];
Mi = [0, 0, 0.093];
rVtNull = [rotationMatrixToVector(R) t];
N = 1000;
tau = 0.00001;
[rV, T] = LevenbergMardquardt(rVtNull, intrinsicMatrix, mapedVertex, imagePointsM,N, tau);
newR1 = rotationVectorToMatrix(rV);
m1 = intrinsicMatrix'*(newR1'*Mi(1,:)'+T');
m1(1)/m1(3)
m1(2)/m1(3)
[rV, T] = IRLS(rVtNull, intrinsicMatrix, mapedVertex, imagePointsM,N, tau);
newR2 = rotationVectorToMatrix(rV);
m2 = intrinsicMatrix'*(newR2'*Mi(1,:)'+T');
m2(1)/m2(3)
m2(2)/m2(3)




























