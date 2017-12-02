data = load('worldToImageCorrespondences.mat');

%%Estimate the world camera pose.
[R, t] = extrinsics(data.imagePoints,data.worldPoints,data.cameraParams);
[worldOrientation,worldLocation] = estimateWorldCameraPose(...
     data.imagePoints,data.worldPoints,data.cameraParams);

%%Plot the world points.
 pcshow(data.worldPoints,'VerticalAxis','Y','VerticalAxisDir','down', ...
     'MarkerSize',30);
 hold on
 plotCamera('Size',10,'Orientation',worldOrientation,'Location',...
     worldLocation);
 hold off
 C = [615 0  320;
       0 615 240;
       0  0   1];
 %P = data.cameraParams.IntrinsicMatrix*RT;
 %P = C*RT;
 R_trans = worldOrientation';
 t_trans = -worldLocation*worldOrientation';
 RT = [R_trans' t_trans'];
 M = [0 0 -10 1];
 worldPoints = [0 0 -10];
 m = M*RT*data.cameraParams.IntrinsicMatrix;
 imgagePoint = worldToImage(data.cameraParams,R,t,worldPoints);