function [relevantF, relevantD, relevantF3D] = pictureDSC_9746(singleVertex,faces,cameraParameters)

% Geting Corner points
DCS_9746 = imread('data/images/init_texture/DSC_9746.jpg');
DSC_9746Gray = rgb2gray(DCS_9746);
point1 = detectMinEigenFeatures(DSC_9746Gray,'ROI',[2073,905,6,6]);
point2 = detectMinEigenFeatures(DSC_9746Gray,'ROI',[1459,1098,6,6]);
point3 = detectMinEigenFeatures(DSC_9746Gray,'ROI', [1709,1198,10,10]);
point4 = detectMinEigenFeatures(DSC_9746Gray,'ROI',[2314,973,8,8]);
point5 = detectMinEigenFeatures(DSC_9746Gray,'ROI',[1477,1563,6,6]);
point6 = detectMinEigenFeatures(DSC_9746Gray,'ROI', [1714,1685,10,10]);
point7 = detectMinEigenFeatures(DSC_9746Gray,'ROI', [2283,1394,6,6]);

% image(DCS_9746);
% hold on;
%plot(point1.selectStrongest(50));
%plot(point2.selectStrongest(50));
%plot(point3.selectStrongest(50));
%plot(point4.selectStrongest(50));
%plot(point5.selectStrongest(50));
%plot(point6.selectStrongest(50));
%plot(point7.selectStrongest(50));
%plot(centerPoint);
% hold off
imagePointsM = [point1.Location;point2.Location;point3.Location;point4.Location;point5.Location;point6.Location;point7.Location];
mapedVertex = [singleVertex(4,:);singleVertex(3,:);singleVertex(2,:);singleVertex(1,:);singleVertex(7,:);singleVertex(6,:);singleVertex(5,:)];
visebleFaces = [faces(5,:);faces(6,:);faces(1,:);faces(2,:);faces(4,:);faces(3,:)];
%% Estimating camera pose
[worldOrientation,worldLocation, inlierIdx] = estimateWorldCameraPose(imagePointsM,mapedVertex,cameraParameters,'MaxReprojectionError',3); 
R = worldOrientation';
t = -worldLocation*worldOrientation';
RT = [R; t];

%%Finding sift
 DSC_9746Single = im2single(DSC_9746Gray);
 [F,D] = vl_sift( DSC_9746Single);
 
 relevantF = [];
 relevantD = [];
 relevantF3D = [];

 [rows, columns] = size(F);
 [rowVisebleFaces, columnsVisebleFaces] = size(visebleFaces);
 for column = 1:columns
     if (F(1,column) >= 1418) && (F(1,column) <= 2286) && (F(2,column) >= 908) && (F(2,column) <= 1702)
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
 image(DCS_9746);
 hold on;
 vl_plotsiftdescriptor(relevantD,relevantF);
 hold off;

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