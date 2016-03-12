package org.usfirst.frc.team1885.robot.serverdata;

import org.json.JSONException;

import dataclient.DataServerWebClient;
import dataclient.robotdata.autonomous.AutonomousConfig;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotAutonomousConfiguration {

    public static final String URL = "http://" + "ilite-drive.local" + ":8083";

    public static AutonomousConfig pullConfiguration() {
        DataServerWebClient client = new DataServerWebClient(URL);
        AutonomousConfig config = new AutonomousConfig(client, 0, 0, 0, 0);

        try {
            config.update(client.getDirect(config.getCollection(), config.getID()).getJSONArray("docs").getJSONObject(0));
        } catch (JSONException e) {
            DriverStation.reportError("error reading autonomousconfig", true);
        }

        return config;
    }

}
