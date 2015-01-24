package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;

public class ToteLift {

    private MotorState state;

    public ToteLift(MotorState state) {
        this.state = state;
    }
    public void setMotorState(MotorState state) {
        this.state = state;
    }
    public MotorState getMotorState() {
        return state;
    }
    public void updateLift() {
        if (state == MotorState.UP) {
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.TOTE_UPPER_LIMIT_SWITCH).get()) {
                state = MotorState.STOP;
            }
        }
    }
    public void cycle(){
        if (state == MotorState.UP) {
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.TOTE_LOWER_LIMIT_SWITCH).get()) {
                state = MotorState.STOP;
            }
        }
    }
}