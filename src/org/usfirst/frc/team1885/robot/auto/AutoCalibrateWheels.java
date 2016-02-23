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
    
    public AutoCalibrateWheels(double rotations){
        initialTickRight = initialTickLeft = currentTickRight = currentTickLeft = 0;
        yawChange = 0;
        P = 2.0;
        I = 0.0002;
        D = 0;
        this.rotations = rotations;
        this.wheelDiameter = 0;
    }
    
    @Override
    public boolean init() {
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).changeControlMode(TalonControlMode.Position);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).changeControlMode(TalonControlMode.Position);
        
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).setPID(P, I, D);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).setPID(P, I, D);
        
        initialTickLeft = SensorInputControlSRX.getInstance().getEncoderPos(SensorType.LEFT_ENCODER);
        initialTickRight = SensorInputControlSRX.getInstance().getEncoderPos(SensorType.RIGHT_ENCODER);
        
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).set(initialTickLeft + (rotations * 1024));
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).set(initialTickRight + (rotations * 1024));
        return true;
    }

    @Override
    public boolean execute() {
        currentTickRight = SensorInputControlSRX.getInstance().getEncoderPos(SensorType.RIGHT_ENCODER);
        currentTickLeft = SensorInputControlSRX.getInstance().getEncoderPos(SensorType.LEFT_ENCODER);
        DriverStation.reportError("\nCurrent Ticks: " + currentTickRight + "  Initial Ticks: " + initialTickRight, false);
        if(currentTickRight - initialTickRight >= (rotations * 1024) && currentTickLeft - initialTickLeft >= (rotations * 1024)){
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
