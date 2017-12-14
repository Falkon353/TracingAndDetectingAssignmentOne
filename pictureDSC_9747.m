function [relevantF, relevantD, relevantF3D] = pictureDSC_9747(singleVertex,faces,cameraParameters)

% Geting Corner points
picture = imread('data/images/init_texture/DSC_9747.jpg');
pictureGray = rgb2gray(picture);
point1 = detectMinEigenFeatures(pictureGray,'ROI',[2240,1006,7,7]);
point2 = detectMinEigenFeatures(pictureGray,'ROI',[1354,992,10,10]);
point3 = detectMinEigenFeatures(pictureGray,'ROI', [1306,1118,8,8]);
point4 = detectMinEigenFeatures(pictureGray,'ROI',[2291,1124,8,8]);
point5 = detectMinEigenFeatures(pictureGray,'ROI',[1339,1594,6,6]);
point6 = detectMinEigenFeatures(pictureGray,'ROI', [2259,1605,8,8]);


% image(picture);
% hold on;
%plot(point1.selectStrongest(50));
%plot(point2.selectStrongest(50));
%plot(point3.selectStrongest(50));
%plot(point4.selectStrongest(50));
%plot(point5.selectStrongest(50));
% plot(point6.selectStrongest(50));
% % %plot(centerPoint);
% hold off
imagePointsM = [point1.Location;point2.Location;point3.Location;point4.Location;point5.Location;point6.Location];
mapedVertex = [singleVertex(4,:);singleVertex(3,:);singleVertex(2,:);singleVertex(1,:);singleVertex(6,:);singleVertex(5,:)];
visebleFaces = [faces(5,:);faces(6,:);faces(1,:);faces(2,:)];
%% Estimating camera pose
[worldOrientation,worldLocation, inlierIdx] = estimateWorldCameraPose(imagePointsM,mapedVertex,cameraParameters,'MaxReprojectionError',5); 
R = worldOrientation';
t = -worldLocation*worldOrientation';
RT = [R; t];

%%Finding sift
 pictureSingle = im2single(pictureGray);
 [F,D] = vl_sift( pictureSingle);
 
 relevantF = [];
 relevantD = [];
 relevantF3D = [];

 [rows, columns] = size(F);
 [rowVisebleFaces, columnsVisebleFaces] = size(visebleFaces);
 for column = 1:columns
     if (F(1,column) >= 1340) && (F(1,column) <= 2256) && (F(2,column) >= 1001) && (F(2,column) <= 1608)
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
 image(picture);
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