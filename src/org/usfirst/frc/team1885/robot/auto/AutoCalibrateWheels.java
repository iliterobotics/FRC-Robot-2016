package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

public class AutoCalibrateWheels extends AutoCommand{

    private final double P, I, D;
    private final double WAIT, ERROR;
    private double initialTickRight, initialTickLeft;
    private double currentTickRight, currentTickLeft;
    private double rotations;
    private double yawChange;
    private double wheelDiameter;
    private long timeLastCheck;
    RobotControlWithSRX robotControl;
    
    public AutoCalibrateWheels(double rotations){
        WAIT = 1000;
        ERROR = 20;
        robotControl = RobotControlWithSRX.getInstance();
        initialTickRight = initialTickLeft = currentTickRight = currentTickLeft = 0;
        yawChange = 0;
        P = 0.7;
        I = 0.0005;
//        P = 0.5;
//        I = 0.0000;
        D = 0;
        this.rotations = rotations;
        this.wheelDiameter = 0;
        timeLastCheck = System.currentTimeMillis();
    }
    
    @Override
    public boolean init() {
        robotControl.getTalons().get(RobotMotorType.LEFT_DRIVE).changeControlMode(TalonControlMode.Position);
        robotControl.getTalons().get(RobotMotorType.RIGHT_DRIVE).changeControlMode(TalonControlMode.Position);
        
        robotControl.getTalons().get(RobotMotorType.LEFT_DRIVE).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        robotControl.getTalons().get(RobotMotorType.RIGHT_DRIVE).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        
        robotControl.getTalons().get(RobotMotorType.LEFT_DRIVE).setPID(P, I, D);
        robotControl.getTalons().get(RobotMotorType.RIGHT_DRIVE).setPID(P, I, D);
        
        initialTickLeft = robotControl.getTalons().get(RobotMotorType.LEFT_DRIVE).get();
        initialTickRight = robotControl.getTalons().get(RobotMotorType.RIGHT_DRIVE).get();
        
        robotControl.getTalons().get(RobotMotorType.LEFT_DRIVE).set(initialTickLeft + 1024);
        robotControl.getTalons().get(RobotMotorType.RIGHT_DRIVE).set(initialTickRight + (1024));
//        DriverStation.reportError("\nLeft Goal:: " + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).get() + "Left Current:: " + initialTickLeft, false);
        return true;
    }

    @Override
    public boolean execute() {
        currentTickRight = robotControl.getTalons().get(RobotMotorType.RIGHT_DRIVE).get();
        currentTickLeft = robotControl.getTalons().get(RobotMotorType.LEFT_DRIVE).get();
//        DriverStation.reportError("\n Right:: " + (initialTickRight + 1024) + " Left:: " + (initialTickLeft + 1024), false);
        DriverStation.reportError("\nLeft: " + (currentTickLeft - initialTickLeft) + " Right: " + (currentTickRight - initialTickRight), false);
        if(withinRange(currentTickRight, currentTickLeft)){
            if(System.currentTimeMillis() - timeLastCheck > WAIT){
                yawChange = SensorInputControlSRX.getInstance().getYaw();
              wheelDiameter = (1.0 * Math.toRadians(yawChange) * AutoAlign.TURN_RADIUS) / Math.PI;
//              wheelDiameter /= rotations;
              DriverStation.reportError("\nChange in Yaw: " + yawChange + "\nWheel Diameter" + wheelDiameter, false);
              return true;  
            }
//          return false;
        } else{
            timeLastCheck = System.currentTimeMillis();
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        return false;
    }

    @Override
    public void reset() {
        
    }
    
    public boolean withinRange(double currentTickRight, double currentTickLeft){
        if(Math.abs(Math.abs(currentTickRight) - Math.abs(initialTickRight)) >= ((rotations * 1024) - ERROR) && Math.abs(Math.abs(currentTickRight) - Math.abs(initialTickRight)) <= ((rotations * 1024) + ERROR)){
            if(Math.abs(Math.abs(currentTickLeft) - Math.abs(initialTickLeft)) >= ((rotations * 1024) - ERROR) && Math.abs(Math.abs(currentTickLeft) - Math.abs(initialTickLeft)) <= ((rotations * 1024) + ERROR)){
                return true;
            }
        }
        return false;
    }

}
