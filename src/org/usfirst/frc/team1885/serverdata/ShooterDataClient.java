package org.usfirst.frc.team1885.serverdata;

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
    
    public HighGoal getData(){
        return highGoalData;
    }
    
    private void updateData(){
        //TODO called whenever data is updated
    }
}