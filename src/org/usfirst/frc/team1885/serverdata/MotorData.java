package org.usfirst.frc.team1885.serverdata;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import dataclient.DataServerWebClient;
import dataclient.robotdata.Motor;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;

public class MotorData implements Runnable{

    private static MotorData instance;
    private static Thread instanceThread;
    
    public static synchronized void start(){
        if(instanceThread != null){
            DriverStation.reportError("motor data client aldready started!!!", false);
        }
        else{
            instance = new MotorData();
            instanceThread = new Thread(instance, "Motor data sending thread");
            instanceThread.start();
        }
    }
    
    public static synchronized void stop(){
        instance.halt();
    }

    
    private boolean running;
    private DataServerWebClient client;
    private Map<RobotMotorType, Motor> motors;
    
    public MotorData(){
        running = true;
        client = new DataServerWebClient(ServerInformation.LAPTOP_IP_ADDRESS);
        motors = new HashMap<RobotMotorType, Motor>();
        RobotControlWithSRX.getInstance().getTalons().entrySet().forEach(entry -> motors.put(entry.getKey(), new Motor(entry.getKey(), client)));
    }
    
    @Override
    public void run() {
        while(running){
            try {
                Thread.sleep(100);
                for(Entry<RobotMotorType, CANTalon> talonEntry : RobotControlWithSRX.getInstance().getTalons().entrySet()){
                    Motor motor = motors.get(talonEntry.getKey());
                    motor.setCurrent(talonEntry.getValue().getOutputCurrent());
                    motor.setVoltage(talonEntry.getValue().getOutputVoltage());
                    motor.setVelocity(talonEntry.getValue().getEncPosition());
                    motor.setPosition(talonEntry.getValue().getEncVelocity());
                    try {
                        motor.push();
                    } catch (IOException e) {
                        DriverStation.reportError("ERROR PUSHING MOTOR:" + motor.getID() + " TO WEBSERVER", false);
                    }
                }
            } catch (InterruptedException e) {
                running = false;
                continue;
            }
        }
    }
    
    public void halt(){
        running = false;
    }

}
