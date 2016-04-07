package org.usfirst.frc.team1885.robot.serverdata;

import java.io.BufferedWriter;
import java.net.InetAddress;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

import org.json.JSONObject;
import org.usfirst.frc.team1885.serverdata.ServerInformation;

import dataclient.DataClient;
import dataclient.DataServerWebClient;
import dataclient.NetworkTablesClient;
import dataclient.robotdata.autonomous.AutonomousConfig;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotAutonomousConfiguration {
    public static AutonomousConfig pullConfiguration() {
        return pullConfiguration(ServerInformation.LAPTOP_HOSTNAME_ADDRESS);
    }
    
    public static AutonomousConfig pullNetworktableConfiguration(){
        DataClient client = new NetworkTablesClient("shooter", false);
        AutonomousConfig config = new AutonomousConfig(client);
        config.update(client.getDirect(config.getCollection(), config.getID()));
        return config;
    }

    private static AutonomousConfig pullConfiguration(String URL){
        AutonomousConfig config = null;
        debugStatement(null, "Entering pullConfiguraton");
        Path path = Paths.get("/robotOutput.txt");
        try (BufferedWriter writer = Files.newBufferedWriter(path,StandardOpenOption.APPEND, StandardOpenOption.WRITE)) {
            InetAddress byAddress = InetAddress.getByAddress(new byte [] {10,18,85,5});
            boolean reachable = byAddress.isReachable(500);
            debugStatement(writer, "Is address reachable " + reachable,null); 
            debugStatement(writer, "trying url:" + URL, null);
//            DataServerWebClient client = new DataServerWebClient(URL);
            DataClient client = new NetworkTablesClient(ServerInformation.TBL_NAME, false);
            config = new AutonomousConfig(client, 0, 0, 0, 0);
            config.setDoingNothing(true);
            config.setShooting(false);
            JSONObject direct = client.getDirect(config.getCollection(), config.getID());
            if(direct == null) {
                debugStatement(writer, "NO DIRECT");
            }
            JSONObject data = client.getDirect(config.getCollection(), config.getID()).getJSONArray("docs").getJSONObject(0);
            if(data == null){
                debugStatement(writer, "My URL= " + URL + " FAILED");
                DriverStation.reportError("Webserver unavailable!", false);
                if(URL.equals(ServerInformation.LAPTOP_IP_ADDRESS)){
                    
                    config =  pullConfiguration(ServerInformation.LAPTOP_HOSTNAME_ADDRESS);
                } else {
                    debugStatement(writer, "DATA FROM SERVER IS NULL!");
                    config = null;
                }
            }
            else{
                DriverStation.reportError("Json:" + data.toString(), false);
                debugStatement(writer, "My URL= " + URL + " SUCCEEDED!");
                config.update(data);
            }
        } catch(Throwable e) {
            debugStatement(null, "Caught exception: ",e);
        } finally {
            debugStatement(null, "Finished pulling configuation= ! " + config);
        }
        
        return config;        
    }

    private static void debugStatement(BufferedWriter bos, String string) {
        debugStatement(bos, string, null);
        
    }

    private static void debugStatement(BufferedWriter bos, String string, Throwable e2) {
//        StringBuilder output = new StringBuilder();
//        output.append(string);
//        if(e2 != null) {
//            StringWriter sw = new StringWriter();
//            PrintWriter pw = new PrintWriter(sw);
//            e2.printStackTrace(pw);
//            output.append("\n");
//            output.append(sw.toString());
//        }
//
//        DriverStation.reportError(output.toString(), false);
//        if(bos != null) {
//            try {
//                bos.write(output.toString());
//                bos.newLine();
//                bos.flush();
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//        }

    }

}
