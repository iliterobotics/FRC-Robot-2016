package org.usfirst.frc.team1885.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class RecycleBinLift {
	private MotorState state;
	private DigitalInput upperLimit;
	private DigitalInput lowerLimit;
	private int rBinCount;

	public RecycleBinLift( MotorState state ) {
		this.state = state;
		rBinCount = 0;
	}

	public MotorState getMotorState() {
		return state;
	}

	public void checkLimit(){
		if( upperLimit.get() ){	
			if( state == MotorState.UP )
				rBinCount++;
			state = MotorState.STOP;
		}

		if( lowerLimit.get() ){
			if( state == MotorState.DOWN )
				rBinCount--;
		} 
	}

}