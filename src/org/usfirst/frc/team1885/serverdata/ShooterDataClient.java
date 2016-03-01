package org.usfirst.frc.team1885.serverdata;

import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import dataclient.DataServerWebClient;
import dataclient.robotdata.vision.HighGoal;

public class ShooterDataClient {
    
    private DataServerWebClient client;
    private HighGoal highGoalData;
    
    public static ShooterDataClient startShooterDataClient(){
        return new ShooterDataClient();
    }
    
    private ShooterDataClient(){
        client = new DataServerWebClient(ServerInformation.DEFAULT_LAPTOP_WEB_ADDRESS);
        highGoalData = new HighGoal(client);
        client.watch(highGoalData, updateData -> updateData());
    }
    
    
    private void updateData(){
        //TODO calculate tilt and twist
        double tiltAngle = 0;
        double twistAngle = 0;
        RobotControlWithSRX.getInstance().updateShooterTilt(tiltAngle);
        RobotControlWithSRX.getInstance().updateShooterTilt(twistAngle);
    }
}
