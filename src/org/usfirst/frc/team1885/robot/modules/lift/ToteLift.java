package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;

public class ToteLift {

	private MotorState state;
	//temp val for the maximum number of totes we can carry
	private final int MAX_TOTE_COUNT = 6;
	private int toteCount;

	public ToteLift(MotorState state) {
		this.state = state;
		toteCount = 0;
	}
	public void setMotorState(MotorState state) {
		this.state = state;
	}
	public MotorState getMotorState() {
		return state;
	}
	public int getToteCount() {
		return toteCount;
	}
	public void updateManualLift() {
		if (state == MotorState.UP) {
			if (toteCount == MAX_TOTE_COUNT) {
				state = MotorState.STOP;
			}
			if (SensorInputControl.getInstance()
					.getLimitSwitch(SensorType.TOTE_LEFT_LIMIT_SWITCH).get()) {
				toteCount++;
			}
		}
		if (state == MotorState.DOWN) {
			if (SensorInputControl.getInstance()
					.getLimitSwitch(SensorType.TOTE_RIGHT_LIMIT_SWITCH).get()) {
				if (toteCount > 0) {
					toteCount--;
				}
			}
		}
	}
}