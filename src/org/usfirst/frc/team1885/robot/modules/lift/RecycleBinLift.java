package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;

public class RecycleBinLift {

    private MotorState state;
    private boolean hasBin;

    public RecycleBinLift(MotorState state) {
        this.state = state;
        hasBin = false;
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
    public void updateLift() {
        if (state == MotorState.UP) {
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.RECYCLE_BIN_UPPER_LIMIT).get()) {
                hasBin = true;
                state = MotorState.STOP;
            }
        }
        if (state == MotorState.DOWN) {
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.RECYCLE_BIN_LOWER_LIMIT).get()) {
                hasBin = false;
                state = MotorState.STOP;
            }
        }
    }
}