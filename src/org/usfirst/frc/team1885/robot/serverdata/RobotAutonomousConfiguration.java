package org.usfirst.frc.team1885.robot.serverdata;

import dataclient.DataServerWebClient;
import dataclient.robotdata.autonomous.AutonomousConfig;

public class RobotAutonomousConfiguration {
    
    public static final String URL = "http://" + "169.254.96.238" + ":8083";
    
    public static AutonomousConfig pullConfiguration(){
        DataServerWebClient client = new DataServerWebClient(URL);
        AutonomousConfig config = new AutonomousConfig(client, 0, 0, 0, 0);
        
        config.update(client.getDirect(config.getCollection(), config.getID()));
        
        return config;
    }
    
}
