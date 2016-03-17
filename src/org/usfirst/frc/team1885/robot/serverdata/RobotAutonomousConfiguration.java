package org.usfirst.frc.team1885.robot.serverdata;

import org.json.JSONException;
import org.json.JSONObject;
import org.usfirst.frc.team1885.serverdata.ServerInformation;

import dataclient.DataServerWebClient;
import dataclient.robotdata.autonomous.AutonomousConfig;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotAutonomousConfiguration {

    public static AutonomousConfig pullConfiguration() {
        DataServerWebClient client = new DataServerWebClient(ServerInformation.DEFAULT_LAPTOP_WEB_ADDRESS);
        AutonomousConfig config = new AutonomousConfig(client, 0, 0, 0, 0);
        config.setDoingNothing(true);
        config.setShooting(false);
        try {
            JSONObject data = client.getDirect(config.getCollection(), config.getID()).getJSONArray("docs").getJSONObject(0);
            if(data == null){
                DriverStation.reportError("Webserver unavailable!", false);
            }
            else{
                config.update(client.getDirect(config.getCollection(), config.getID()).getJSONArray("docs").getJSONObject(0));
            }
        } catch (Exception e) {
            DriverStation.reportError("error reading autonomousconfig", true);
        }

        return config;
    }

}
