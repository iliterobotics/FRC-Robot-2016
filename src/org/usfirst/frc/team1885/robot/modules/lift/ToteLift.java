package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;

public class ToteLift {

    private MotorState state;
    private int hookPass;

    public ToteLift(MotorState state) {
        this.state = state;
        hookPass = 0;
    }
    public void setMotorState(MotorState state) {
        this.state = state;
    }
    public MotorState getMotorState() {
        return state;
    }
    public void updateManualLift() {
        if (state == MotorState.UP) {
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.TOTE_UPPER_LIMIT_SWITCH).get()) {
                state = MotorState.STOP;
            }
        }
        if (SensorInputControl.getInstance()
                .getLimitSwitch(SensorType.TOTE_LOWER_LIMIT_SWITCH).get()) {
            if (state == MotorState.UP) {
                hookPass++;
            }
            if (state == MotorState.DOWN) {
                hookPass--;
            }
        }
    }
}