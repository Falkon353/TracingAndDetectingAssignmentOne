clear all;
run('vlfeat-0.9.20/toolbox/vl_setup');
[vertex,faces] =  read_ply('data/model/teabox.ply');
faces = faces+1;
singleVertex = single(vertex);

intrinsicMatrix = [2960.37845     0         0;
                       0      2960.37845    0;
                   1841.68855 1235.23369    1];

cameraParameters =  cameraParameters('IntrinsicMatrix',intrinsicMatrix);
 
[relevantFPic1, relevantDPic1, relevantF3DPic1] = pictureDSC_9743(singleVertex,faces,cameraParameters);
[relevantFPic2, relevantDPic2, relevantF3DPic2] = pictureDSC_9744(singleVertex,faces,cameraParameters);
[relevantFPic3, relevantDPic3, relevantF3DPic3] = pictureDSC_9745(singleVertex,faces,cameraParameters);
[relevantFPic4, relevantDPic4, relevantF3DPic4] = pictureDSC_9746(singleVertex,faces,cameraParameters);
[relevantFPic5, relevantDPic5, relevantF3DPic5] = pictureDSC_9747(singleVertex,faces,cameraParameters);
 
relevantF = [relevantFPic1 relevantFPic2 relevantFPic3 relevantFPic4];
relevantD = [relevantDPic1 relevantDPic2 relevantDPic3 relevantDPic4];
relevantF3D = [relevantF3DPic1; relevantF3DPic2; relevantF3DPic3; relevantF3DPic4];

[worldOrientation, worldLocation] = RansacFunction(relevantD, relevantF3D,intrinsicMatrix, 'data/images/init_texture/DSC_9743.jpg');
R = worldOrientation';
t = -worldLocation*worldOrientation';
RT = [R; t];
picture = imread('data/images/init_texture/DSC_9743.jpg');
image(picture);
hold on;
for i = 1:8
    M = [vertex(i,:) 1];
    m = M*RT*cameraParameters.IntrinsicMatrix;
    m1 = m(1)/m(2)
    m2 = m(2)/m(3)
    plot(m(1)/m(3),m(2)/m(3));
end
hold off;

