package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ActiveIntake implements Module{

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
    public void setMotorState(MotorState state) {
        this.state = state;
    }
    public MotorState getMotorState() {
        return state;
    }
    public double getSpeed() {
        return intakeSpeed;
    }
    public void updateIntake() {
        
        if ((DriverInputControl.getInstance().getButton(
                RobotButtonType.INTAKE_IN))) {
                state = MotorState.REVERSE;
                intakeSpeed = -.5;
            }
    
        if ((DriverInputControl.getInstance().getButton(
                RobotButtonType.INTAKE_OUT))) {
                state = MotorState.FORWARD;
                intakeSpeed = .5;
            }
        updateIntake(.5);
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