package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;

public class RecycleBinLift {

    public static final double DEFAULT_LIFT_SPEED = .5;
    private static RecycleBinLift instance;
    private double liftSpeed;
    private boolean hasBin;
    private MotorState state;

    protected RecycleBinLift() {
        this.state = MotorState.STOP;
        hasBin = false;
        liftSpeed = 0;
        
        // TODO fix me
        SensorInputControl.getInstance().getEncoder(SensorType.RECYCLE_BIN_ENCODER).setDistancePerPulse(1/360);
    }
    public static RecycleBinLift getInstance() {
        if (instance == null) {
            instance = new RecycleBinLift();
        }
        return instance;
    }
    public void setMotorState(MotorState state) {
        this.state = state;
    }
    public MotorState getMotorState() {
        return state;
    }
    public boolean hasBin() {
        return hasBin;
    }
    public double getSpeed() {
        return liftSpeed;
    }
    public void updateLift() {
        if (state == MotorState.UP) {
            liftSpeed = DEFAULT_LIFT_SPEED;
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.RECYCLE_BIN_UPPER_LIMIT).get()) {
                hasBin = true;
                stop();
            }
        }
        if (state == MotorState.DOWN) {
            liftSpeed = -DEFAULT_LIFT_SPEED;
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.RECYCLE_BIN_LOWER_LIMIT).get()) {
                hasBin = false;
                stop();
            }
        }
    }
    public void updateLift(double speed) {
        liftSpeed = speed;
        if (speed > 0) {
            state = MotorState.UP;
        } else if (speed < 0) {
            state = MotorState.DOWN;
        } else {
            state = MotorState.STOP;
        }
        if (state == MotorState.UP) {
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.RECYCLE_BIN_UPPER_LIMIT).get()) {
                hasBin = true;
                stop();
            }
        }
        if (state == MotorState.DOWN) {
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.RECYCLE_BIN_LOWER_LIMIT).get()) {
                hasBin = false;
                stop();
            }
        }
    }
    public void stop() {
        state = MotorState.STOP;
        liftSpeed = 0;
    }
}