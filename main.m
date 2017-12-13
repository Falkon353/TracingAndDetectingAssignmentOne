clear all;
run('vlfeat-0.9.20/toolbox/vl_setup');
[vertex,faces] =  read_ply('data/model/teabox.ply');
faces = faces+1;
singleVertex = single(vertex);

intrinsicMatrix = [2960.37845     0         0;
                       0      2960.37845    0;
                   1841.68855 1235.23369    1];

cameraParameters =  cameraParameters('IntrinsicMatrix',intrinsicMatrix);
%  relevantF = [];
%  relevantD = [];
%  relevantF3D = [];
 
 %[relevantFPic1, relevantDPic1, relevantF3DPic1] = pictureDSC_9743(singleVertex,faces,cameraParameters);
 %[relevantFPic2, relevantDPic2, relevantF3DPic2] = pictureDSC_9744(singleVertex,faces,cameraParameters);
 [relevantFPic2, relevantDPic2, relevantF3DPic2] = pictureDSC_9746(singleVertex,faces,cameraParameters);