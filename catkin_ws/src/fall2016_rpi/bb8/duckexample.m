duck=RobotRaconteur.Connect('tcp://bb8.local:1234/DuckiebotServer.bb8/Duckiebot');

n = 921600; h = 640; w = 480;

tic;

I1 = duck.getImage();

C = zeros(480,640,3,'uint8');
C(:,:,1)=reshape(I1.data(1:3:n),h,w)';
C(:,:,2)=reshape(I1.data(2:3:n),h,w)';
C(:,:,3)=reshape(I1.data(3:3:n),h,w)';

toc
figure(1);image(C);
colorbar