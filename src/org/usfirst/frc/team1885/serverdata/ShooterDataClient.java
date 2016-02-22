package org.usfirst.frc.team1885.serverdata;

import dataclient.DataServerWebClient;
import dataclient.robotdata.vision.HighGoal;

public class ShooterDataClient {
    
    private static final String DEFAULT_LAPTOP_WEB_ADDRESS = "http://10.18.85.207:8083";
    
    private DataServerWebClient client;
    private HighGoal highGoalData;
    
    public ShooterDataClient(){
        client = new DataServerWebClient(DEFAULT_LAPTOP_WEB_ADDRESS);
        highGoalData = new HighGoal(client);
        client.watch(highGoalData, updateData -> updateData());
    }
    
    private void updateData(){
        
    }
}
