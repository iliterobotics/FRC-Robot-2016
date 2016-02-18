package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.TruePID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.RobotDrive.MotorType;

public class AutoDriveForward {
    
    private static final double ALLOWABLE_MOTOR_DIFF = 0.05; //in percentage
    
    private TruePID leftPID;
    private TruePID rightPID;
    
    private int initialLeftTicks;
    private int initialRightTicks;
    
    private static final double P = 0.5;
    private static final double I = 0.5;
    private static final double D = 0.5;
    private static final int FULL_ERROR = 50;
    
    private final SensorType LEFT_ENCODER;
    private final SensorType RIGHT_ENCODER;
    
    private final MotorType LEFT_MOTOR;
    private final MotorType RIGHT_MOTOR;
    
    private RobotControlWithSRX srx;
    
    /**generic constructor*/
    public AutoDriveForward(SensorType leftEncoder, SensorType rightEncoder, MotorType leftMotor, MotorType rightMotor){
        LEFT_ENCODER = leftEncoder;
        RIGHT_ENCODER = rightEncoder;
        
        LEFT_MOTOR = leftMotor;
        RIGHT_MOTOR = rightMotor;
        
        leftPID = new TruePID(P, I, D, FULL_ERROR);
        rightPID = new TruePID(P, I, D, FULL_ERROR);
        
        srx = RobotControlWithSRX.getInstance();
    }
    
    public void checkDriveForward(double leftMotor, double rightMotor){
        if(Math.abs(leftMotor - rightMotor) < ALLOWABLE_MOTOR_DIFF){
            
            if(initialLeftTicks == -1){
                initialLeftTicks = srx.getSensor().get(LEFT_ENCODER).getEncPosition();
            }
            if(initialRightTicks == -1){
                initialRightTicks = srx.getSensor().get(RIGHT_ENCODER).getEncPosition();
            }
            
            int leftTicks = srx.getSensor().get(LEFT_ENCODER).getEncPosition();
            int rightTicks = srx.getSensor().get(RIGHT_ENCODER).getEncPosition();
            
            
            if(leftTicks > rightTicks){
                double pidAdjustment = rightPID.getPID(leftTicks, rightTicks);
                if(rightMotor + pidAdjustment > 1){
                    leftMotor += leftPID.getPID(leftTicks, rightTicks);
                }
                else
                    rightMotor += pidAdjustment;
            }
            else if(rightTicks > leftTicks){
                double pidAdjustment = leftPID.getPID(leftTicks, rightTicks);
                if(leftMotor + pidAdjustment > 1){
                    rightMotor += rightPID.getPID(leftTicks, rightTicks);
                }
                else
                    leftMotor += pidAdjustment;
            }
            
            srx.getTalons().get(LEFT_MOTOR).set(leftMotor);
            srx.getTalons().get(RIGHT_MOTOR).set(rightMotor);
        }
        else{
            leftPID.reset();
            rightPID.reset();
            
            initialLeftTicks = -1;
            initialRightTicks= -1;
        }
    }
}
