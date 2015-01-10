package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.MotorState;

public class ToteLift {
	private MotorState state;
	private int toteCount;

	public ToteLift(MotorState state) {
		this.state = state;
		toteCount = 0;
	}

	public MotorState getMotorState() {
		return state;
	}

//	public void checkLimit() {
//		if(crateLiftLimitTop.get()) {	
//			state = MotorState.STOP;
//			toteCount++;
//		}
//
//		if(crateLiftLimitBottom.get()){
//			if(state == MotorState.DOWN) {
//				toteCount--;
//			}
//		} 
//	}
}