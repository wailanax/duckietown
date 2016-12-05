duck=RobotRaconteur.Connect('tcp://bb8.local:1234/DuckiebotServer.bb8/Duckiebot');
ap = duck.april_tags;
tic
while (1) 
   ap = duck.april_tags;
   pos_x = ap{1,1}.pos(1)
   pos_y = ap{1,1}.pos(2)
   pos_z = ap{1,1}.pos(3)
   pause(0.3);
   %toc
end