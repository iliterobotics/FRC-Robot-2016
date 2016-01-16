package org.usfirst.frc.team1885.graveyard;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.modules.Module;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ActiveIntake implements Module{

    public static final int INTAKE_SPEED = 1;
    private static ActiveIntake instance;
    private double intakeLeftSpeed;
    private double intakeRightSpeed;
    private MotorState leftState;
    private MotorState rightState;

    protected ActiveIntake() {
        this.leftState = MotorState.OFF;
        this.rightState = MotorState.OFF;
        intakeLeftSpeed = 0; 
        intakeRightSpeed = 0;
    }
    public static ActiveIntake getInstance() {
        if (instance == null) {
            instance = new ActiveIntake();
        }
        return instance;
    }
    public void setMotorState(MotorState leftState, MotorState rightState) {
        this.leftState = leftState;
        this.rightState = rightState;
    }
    public MotorState getLeftMotorState() {
        return leftState;
    }
    public MotorState getRightMotorState(){
        return rightState;
    }
    public double getLeftSpeed() {
        return intakeLeftSpeed;
    }
    public double getRightSpeed(){
        return intakeRightSpeed;
    }
    public void updateIntake() {
        intakeLeftSpeed = 0;
        intakeRightSpeed = 0;
        
        if ((DriverInputControl.getInstance().getButton(
                RobotButtonType.INTAKE_IN))) {
                leftState = MotorState.REVERSE;
                rightState = MotorState.FORWARD;
                intakeLeftSpeed = INTAKE_SPEED;
                intakeRightSpeed = -INTAKE_SPEED;
            }
    
        if ((DriverInputControl.getInstance().getButton(
                RobotButtonType.INTAKE_OUT))) {
                leftState = MotorState.FORWARD;
                rightState = MotorState.REVERSE;
                intakeLeftSpeed = -INTAKE_SPEED;
                intakeRightSpeed = INTAKE_SPEED;
            }
        updateIntake(intakeLeftSpeed, intakeRightSpeed);
    }
    public void updateIntake(double leftSpeed, double rightSpeed) {
        intakeLeftSpeed = leftSpeed;
        intakeRightSpeed = rightSpeed;
        if (leftSpeed > 0) {
            leftState = MotorState.REVERSE;
        } else if (leftSpeed < 0) {
            leftState = MotorState.FORWARD;
        } else {
            leftState = MotorState.OFF;
        }
        if (rightSpeed > 0) {
            rightState = MotorState.REVERSE;
        } else if (rightSpeed < 0) {
            rightState = MotorState.FORWARD;
        } else {
            rightState = MotorState.OFF;
        }
     }
    
    public void stop() {
        leftState = MotorState.OFF;
        intakeLeftSpeed = 0;
        rightState = MotorState.OFF;
        intakeRightSpeed = 0;
    }
    
    public void updateOutputs() {
        RobotControl.getInstance().updateIntakeMotors(intakeLeftSpeed, intakeRightSpeed);
    }
    @Override
    public void update() {
        updateIntake();
    }
}