duck=RobotRaconteur.Connect('tcp://bb8.local:1234/DuckiebotServer.bb8/Duckiebot');

while (1)
    duck.sendCmd(0.5,3.0);
end
duck.sendCmd(0.0,0.0);