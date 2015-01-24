package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;

public class ToteLift {

    public static final double DEFAULT_LIFT_SPEED = .5;
    private static ToteLift instance;
    private double liftSpeed;
    private MotorState state;

    protected ToteLift() {
        this.state = MotorState.STOP;
        liftSpeed = 0;
    }
    public static ToteLift getInstance()
    {
        if(instance == null)
        {
            instance = new ToteLift();
        }
        return instance;
    }
    public void setMotorState(MotorState state) {
        this.state = state;
    }
    public MotorState getMotorState() {
        return state;
    }
    public double getSpeed(){
        return liftSpeed;
    }
    public void updateLift() {
        if (state == MotorState.UP) {
            liftSpeed = DEFAULT_LIFT_SPEED;
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.TOTE_UPPER_LIMIT_SWITCH).get()) {
                stop();
            }
        }
        if(state == MotorState.DOWN){
            liftSpeed = -DEFAULT_LIFT_SPEED;
        }
    }
    public void updateLift(double speed) {
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
                    .getLimitSwitch(SensorType.TOTE_UPPER_LIMIT_SWITCH).get()) {
                stop();
            }
        }
    }
    public void cycle(){
        if (state == MotorState.UP) {
            liftSpeed = DEFAULT_LIFT_SPEED;
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.TOTE_LOWER_LIMIT_SWITCH).get()) {
                stop();
            }
        }
    }
    public void stop(){
        state = MotorState.STOP;
        liftSpeed = 0;
    }
}