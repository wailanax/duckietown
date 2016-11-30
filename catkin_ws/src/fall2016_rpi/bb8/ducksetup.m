if ~exist('duck')
    duck=RobotRaconteur.Connect('tcp://10.0.1.34:1234/DuckiebotServer.duckmobile/Duckiebot');
    camera_on=0;
else
    if camera_on==0;duck.openCamera();camera_on=1;end
    duckkeydrive(duck);
end

