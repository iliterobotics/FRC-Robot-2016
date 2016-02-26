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
    private double initialTickRight, initialTickLeft;
    private double currentTickRight, currentTickLeft;
    private double rotations;
    private double yawChange;
    private double wheelDiameter;
    RobotControlWithSRX robotControl;
    
    public AutoCalibrateWheels(double rotations){
        robotControl = RobotControlWithSRX.getInstance();
        initialTickRight = initialTickLeft = currentTickRight = currentTickLeft = 0;
        yawChange = 0;
        P = 2.0;
        I = 0.0004;
        D = 0;
        this.rotations = rotations;
        this.wheelDiameter = 0;
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
        
        robotControl.getTalons().get(RobotMotorType.LEFT_DRIVE).set(initialTickLeft + (rotations * 1024));
        robotControl.getTalons().get(RobotMotorType.RIGHT_DRIVE).set(initialTickRight + (rotations * 1024));
        DriverStation.reportError("Left Goal:: " + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).get(), false);
        return true;
    }

    @Override
    public boolean execute() {
        currentTickRight = robotControl.getTalons().get(RobotMotorType.RIGHT_DRIVE).get();
        currentTickLeft = robotControl.getTalons().get(RobotMotorType.LEFT_DRIVE).get();
        DriverStation.reportError("\nCurrent Ticks:: Left: " + currentTickLeft + " Right: " + currentTickRight + "  Initial Ticks:: Left: " + initialTickLeft + " Right: " + initialTickRight, false);
        if(Math.abs(currentTickRight) - Math.abs(initialTickRight) >= (rotations * 1024) && Math.abs(Math.abs(currentTickLeft) - Math.abs(initialTickLeft)) >= (rotations * 1024)){
          yawChange = SensorInputControlSRX.getInstance().getYaw();
          wheelDiameter = (1.0 * Math.toRadians(yawChange) * AutoAlign.TURN_RADIUS) / Math.PI;
//          wheelDiameter /= rotations;
          DriverStation.reportError("\nChange in Yaw: " + yawChange + "\nWheel Diameter" + wheelDiameter, false);
          return true;  
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

}