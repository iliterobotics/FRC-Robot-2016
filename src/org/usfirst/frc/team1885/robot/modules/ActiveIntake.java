package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.graveyard.DriverInputControl;
import org.usfirst.frc.team1885.graveyard.RobotControl;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ActiveIntake implements Module{

    public static final int INTAKE_SPEED = 1;
    private static ActiveIntake instance;
    private double intakeSpeed;
    private MotorState state;

    protected ActiveIntake() {
        this.state = MotorState.OFF;
        intakeSpeed = 0;
    }
    public static ActiveIntake getInstance() {
        if (instance == null) {
            instance = new ActiveIntake();
        }
        return instance;
    }
    public void setMotorState(MotorState motorState) {
        this.state = motorState;
    }
    public MotorState getMotorState() {
        return state;
    }
    public double getSpeed(){
        return intakeSpeed;
    }
    public void updateIntake() {
        intakeSpeed = 0;
        
        if ((DriverInputControl.getInstance().getButton(
                RobotButtonType.INTAKE_IN))) {
                state = MotorState.REVERSE;
                intakeSpeed = INTAKE_SPEED;
            }
        updateIntake(intakeSpeed);
    }
    public void updateIntake(double speed) {
        intakeSpeed = speed;
        if (speed > 0) {
            state = MotorState.REVERSE;
        } else if (speed < 0) {
            state = MotorState.FORWARD;
        } else {
            state = MotorState.OFF;
        }
     }
    
    public void stop() {
        state = MotorState.OFF;
        intakeSpeed = 0;
    }
    
    public void updateOutputs() {
        RobotControl.getInstance().updateIntakeMotors(intakeSpeed);
    }
    @Override
    public void update() {
        updateIntake();
    }
}