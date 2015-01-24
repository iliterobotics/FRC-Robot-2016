package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;

public class RecycleBinLift {

    public static final double DEFAULT_LIFT_SPEED = .5;
    private double liftSpeed;
    private MotorState state;
    private boolean hasBin;
    private static RecycleBinLift instance;
    
    protected RecycleBinLift() {
        this.state = MotorState.STOP;
        hasBin = false;
        liftSpeed = 0;
    }
    public static RecycleBinLift getInstance()
    {
        if(instance == null)
        {
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
    public double getSpeed(){
        return liftSpeed;
    }
    public void updateLift() {
        if (state == MotorState.UP) {
            liftSpeed = DEFAULT_LIFT_SPEED;
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.RECYCLE_BIN_UPPER_LIMIT).get()) {
                hasBin = true;
                state = MotorState.STOP;
                liftSpeed = 0;
            }
        }
        if (state == MotorState.DOWN) {
            liftSpeed = -DEFAULT_LIFT_SPEED;
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.RECYCLE_BIN_LOWER_LIMIT).get()) {
                hasBin = false;
                state = MotorState.STOP;
                liftSpeed = 0;
            }
        }
    }
    public void updateLIft( double speed ){
        liftSpeed = speed;
        if( speed > 0 ){
            state = MotorState.UP;
        }else if( speed < 0 ){
            state = MotorState.DOWN;
        }else{
            state = MotorState.STOP;
        }
        if (state == MotorState.UP) {
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.RECYCLE_BIN_UPPER_LIMIT).get()) {
                hasBin = true;
                state = MotorState.STOP;
                liftSpeed = 0;
            }
        }
        if (state == MotorState.DOWN) {
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.RECYCLE_BIN_LOWER_LIMIT).get()) {
                hasBin = false;
                state = MotorState.STOP;
                liftSpeed = 0;
            }
        }
    }
}