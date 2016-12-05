if ~exist('bb8')
    duck=RobotRaconteur.Connect('tcp://bb8.local:1234/DuckiebotServer.bb8/Duckiebot');
    camera_on=0;
else
    if camera_on==0;duck.openCamera();camera_on=1;end
    duckkeydrive(duck);
end

