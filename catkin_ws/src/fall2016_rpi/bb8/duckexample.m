duck=RobotRaconteur.Connect('tcp://bb8.local:1234/DuckiebotServer.bb8/Duckiebot');

duck.sendCmd(1.0,3.0); %the lager omega is, more left the duckbot will go. 
                       %e.g 5.0 is too large
pause(1.5);

duck.sendCmd(0.0,0.0);

%{
duck.sendCmd(0.8,-10);
pause(0.5);
duck.sendCmd(1.0,0.0);
pause(1);
duck.sendCmd(0.0,0.0);
%}