package org.usfirst.frc.team1885.serverdata;

import dataclient.DataClient;
import dataclient.DataServerWebClient;
import dataclient.robotdata.vision.HighGoal;

public class ShooterDataClient {
    
    private static final String TBL_NAME = "shooter";
    private DataClient client;
    private HighGoal highGoalData;
    
    public static ShooterDataClient startShooterDataClient(){
        return new ShooterDataClient();
    }
    
    private ShooterDataClient(){
//        HTTP CODE:
        client = new DataServerWebClient(ServerInformation.LAPTOP_HOSTNAME_ADDRESS);
//        NETWORK TABLES CODE:
//        client = new NetworkTablesClient(TBL_NAME, false);
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
