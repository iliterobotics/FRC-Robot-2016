package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class Shooter implements Module {

    private static Shooter instance;
    
    private final double SHOOTER_SPEED = .2;
    private double flywheelSpeedLeft;
    private MotorState leftState;
    private double flywheelSpeedRight;
    private MotorState rightState;
    private static DriverInputControlSRX driverInputControl;
    
    public static Shooter getInstance(){
        if( instance == null ){
            instance = new Shooter();
        }
        return instance;
    }
    protected Shooter() {
        this.leftState = MotorState.OFF;
        this.rightState = MotorState.OFF;
        flywheelSpeedLeft = 0;
        flywheelSpeedRight = 0;
        driverInputControl = DriverInputControlSRX.getInstance();
    }
    public MotorState getLeftMotorState() {
        return leftState;
    }
    public MotorState getRightMotorState() {
        return rightState;
    }
    public double getLeftSpeed(){
        return flywheelSpeedLeft;
    }
    public double getRightSpeed(){
        return flywheelSpeedRight;
    }
    public void updateShooter() {
        //TODO modify values after testing for direction
        if (driverInputControl.getButton(RobotButtonType.FLYWHEEL_OUT)) {
                flywheelSpeedLeft = -SHOOTER_SPEED;
                flywheelSpeedRight = SHOOTER_SPEED;
        }
        else if (driverInputControl.getButton(RobotButtonType.FLYWHEEL_IN)) {
            flywheelSpeedLeft = SHOOTER_SPEED;
            flywheelSpeedRight = -SHOOTER_SPEED;    
        }
        else {
            flywheelSpeedLeft = 0;
            flywheelSpeedRight = 0;
        }
        updateShooter(flywheelSpeedLeft, flywheelSpeedRight);
    }
    public void updateShooter(double speedLeft, double speedRight) {
        flywheelSpeedLeft = speedLeft;
        flywheelSpeedRight = speedRight;
        if (speedLeft > 0) {
            leftState = MotorState.REVERSE;
        } else if (speedLeft < 0) {
            leftState = MotorState.FORWARD;
        } else {
            leftState = MotorState.OFF;
        }
        if (speedRight > 0) {
            rightState = MotorState.REVERSE;
        } else if (speedRight < 0) {
            rightState = MotorState.FORWARD;
        } else {
            rightState = MotorState.OFF;
        }
     }
    public void reset() {
        rightState = leftState = MotorState.OFF;
        flywheelSpeedRight = flywheelSpeedLeft = 0;
    }
    
    public void updateOutputs() {
        RobotControlWithSRX.getInstance().updateFlywheelShooter(flywheelSpeedLeft, flywheelSpeedRight);
        RobotControlWithSRX.getInstance().updateShooterTilt(driverInputControl.getShooterTilt() * .3);
        RobotControlWithSRX.getInstance().updateShooterTilt(driverInputControl.getShooterTwist() * .3);
    }
    @Override
    public void update() {
        updateShooter();
    }

}
