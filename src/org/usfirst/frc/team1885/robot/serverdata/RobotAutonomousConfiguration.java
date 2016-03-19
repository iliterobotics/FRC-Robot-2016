package org.usfirst.frc.team1885.robot.serverdata;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;

import org.json.JSONObject;
import org.usfirst.frc.team1885.serverdata.ServerInformation;

import dataclient.DataServerWebClient;
import dataclient.robotdata.autonomous.AutonomousConfig;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotAutonomousConfiguration {
    
    private static File mDebugFile = new File("/home/lvuser/robotOutput.txt");

    public static AutonomousConfig pullConfiguration() {
        return pullConfiguration(ServerInformation.LAPTOP_IP_ADDRESS);
    }
    
    private static AutonomousConfig pullConfiguration(String URL){
        FileOutputStream fos = null;
        BufferedWriter bos = null;
        try {
             fos = new  FileOutputStream(mDebugFile);
             bos = new  BufferedWriter(new OutputStreamWriter(fos));
        } catch (FileNotFoundException e1) {
            e1.printStackTrace();
            DriverStation.reportError("NO LOG FILE FOUND", false);
        }
        debugStatement(bos, "trying url:" + URL);
        
        debugStatement(bos, "My URL= " + URL);
        DataServerWebClient client = new DataServerWebClient(URL);
        AutonomousConfig config = new AutonomousConfig(client, 0, 0, 0, 0);
        config.setDoingNothing(true);
        config.setShooting(false);
        try {
            JSONObject data = client.getDirect(config.getCollection(), config.getID()).getJSONArray("docs").getJSONObject(0);
            if(data == null){
                debugStatement(bos, "My URL= " + URL + " FAILED");
                DriverStation.reportError("Webserver unavailable!", false);
                if(URL.equals(ServerInformation.LAPTOP_IP_ADDRESS)){
                    return pullConfiguration(ServerInformation.LAPTOP_HOSTNAME_ADDRESS);
                }
                else{
                    return null;
                }
            }
            else{
                DriverStation.reportError("Json:" + data.toString(), false);
                debugStatement(bos, "My URL= " + URL + " SUCCEEDED!");
                config.update(data);
            }
        } catch (Exception e) {
            DriverStation.reportError("error reading autonomousconfig", true);
            DriverStation.reportError("Error type:" + e.getClass(), false);
            debugStatement(bos, "CAUGHT EXCEPTON: !"+  e.getClass());
            e.printStackTrace();
            try {
                e.printStackTrace(new PrintWriter(mDebugFile));
            } catch (FileNotFoundException e1) {
                e1.printStackTrace();
            }
        } finally {
            if(bos != null) {
                try {
                    bos.close();
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
            if(fos != null) {
                try {
                    fos.close();
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }
        return config;        
    }

    private static void debugStatement(BufferedWriter bos, String string) {
        DriverStation.reportError(string, false);
        if(bos != null) {
            try {
                bos.write(string);
                bos.newLine();
                bos.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        
    }

}
