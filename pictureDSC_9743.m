function [relevantF, relevantD, relevantF3D] = pictureDSC_9743(singleVertex,faces,cameraParameters)

%% Geting Corner points
DSC_9743 = imread('data/images/init_texture/DSC_9743.jpg');
testPictureAGray = rgb2gray(DSC_9743);
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
%% Estimating camera pose
[worldOrientation,worldLocation, inlierIdx] = estimateWorldCameraPose(imagePointsM,mapedVertex,cameraParameters,'MaxReprojectionError',3); 
R = worldOrientation';
t = -worldLocation*worldOrientation';
RT = [R; t];
%%Finding sift
 testPictureASingel = im2single(testPictureAGray);
 [F,D] = vl_sift( testPictureASingel);
 
 relevantF = [];
 relevantD = [];
 relevantF3D = [];

 [rows, columns] = size(F);
 [rowVisebleFaces, columnsVisebleFaces] = size(visebleFaces);
 for column = 1:columns
     if (F(1,column) >= 1377) && (F(1,column) <= 2236) && (F(2,column) >= 1016) && (F(2,column) <= 1611)
         for face = 1:rowVisebleFaces
             dir = inv(cameraParameters.IntrinsicMatrix')*[F(1,column);F(2,column);1];
             unitDir = dir/norm(dir);
             dirRotated = R*unitDir;
             [intersect, distance, u, v, xcoor] = TriangleRayIntersection(worldLocation, dirRotated, singleVertex(visebleFaces(face,1),:)', ...
                 singleVertex(visebleFaces(face,2),:)',singleVertex(visebleFaces(face,3),:)','border', 'inclusive');
             if intersect == 1
                relevantF = [relevantF F(:,column)];
                relevantD = [relevantD D(:,column)];
                relevantF3D = [relevantF3D; xcoor];
             end
         end
     end
 end
 image(DSC_9743);
 hold on;
 vl_plotsiftdescriptor(relevantD,relevantF);
 hold off;
 
 