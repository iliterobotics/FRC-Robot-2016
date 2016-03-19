package org.usfirst.frc.team1885.robot.serverdata;

import java.io.IOException;

import org.json.JSONException;
import org.json.JSONObject;
import org.usfirst.frc.team1885.serverdata.ServerInformation;

import dataclient.DataServerWebClient;
import dataclient.robotdata.autonomous.AutonomousConfig;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotAutonomousConfiguration {

    public static AutonomousConfig pullConfiguration() {
        return pullConfiguration(ServerInformation.DEFAULT_LAPTOP_WEB_ADDRESS);
    }
    
    private static AutonomousConfig pullConfiguration(String URL){
        DataServerWebClient client = new DataServerWebClient(URL);
        AutonomousConfig config = new AutonomousConfig(client, 0, 0, 0, 0);
        config.setDoingNothing(true);
        config.setShooting(false);
        try {
            JSONObject data = client.getDirect(config.getCollection(), config.getID()).getJSONArray("docs").getJSONObject(0);
            if(data == null){
                DriverStation.reportError("Webserver unavailable!", false);
                if(URL.equals(ServerInformation.DEFAULT_LAPTOP_WEB_ADDRESS)){
                    return pullConfiguration(ServerInformation.LAPTOP_HOSTNAME_ADDRESS);
                }
            }
            else{
                config.update(client.getDirect(config.getCollection(), config.getID()).getJSONArray("docs").getJSONObject(0));
            }
        } catch (Exception e) {
            DriverStation.reportError("error reading autonomousconfig", true);
            e.printStackTrace();
        }

        return config;        
    }

}
