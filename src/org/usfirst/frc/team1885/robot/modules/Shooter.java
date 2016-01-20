package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.graveyard.DriverInputControl;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class Shooter implements Module {

    private static Shooter instance;
    
    private final int SHOOTER_SPEED = 1;
    private double shooterSpeedLeft;
    private MotorState leftState;
    private double shooterSpeedRight;
    private MotorState rightState;
    
    public static Shooter getInstance(){
        if( instance == null ){
            instance = new Shooter();
        }
        return instance;
    }
    protected Shooter() {
        this.leftState = MotorState.OFF;
        this.rightState = MotorState.OFF;
        shooterSpeedLeft = 0;
        shooterSpeedRight = 0;
    }
    
    public void setMotorState(MotorState leftMotorState, MotorState rightMotorState) {
        this.leftState = leftMotorState;
        this.rightState = rightMotorState;
    }
    public MotorState getLeftMotorState() {
        return leftState;
    }
    public MotorState getRightMotorState() {
        return rightState;
    }
    public double getLeftSpeed(){
        return shooterSpeedLeft;
    }
    public double getRightSpeed(){
        return shooterSpeedRight;
    }
    public void updateShooter() {
        shooterSpeedLeft = 0;
        shooterSpeedRight = 0;
        //TODO modify values after testing for direction
        if ((DriverInputControl.getInstance().getButton(
                RobotButtonType.SHOOTER_OUT))) {
                leftState = MotorState.REVERSE;
                rightState = MotorState.FORWARD;
                shooterSpeedLeft = -SHOOTER_SPEED;
                shooterSpeedRight = SHOOTER_SPEED;
            }
        updateShooter(shooterSpeedLeft, shooterSpeedRight);
    }
    public void updateShooter(double speedLeft, double speedRight) {
        shooterSpeedLeft = speedLeft;
        shooterSpeedRight = speedRight;
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
    
    public void stop() {
        rightState = leftState = MotorState.OFF;
        shooterSpeedRight = shooterSpeedLeft = 0;
    }
    
    public void updateOutputs() {
        RobotControlWithSRX.getInstance().updateShooterMotors(shooterSpeedLeft, shooterSpeedRight);
    }
    @Override
    public void update() {
        updateShooter();
    }

}
