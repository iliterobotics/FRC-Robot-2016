package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.MotorState;

import edu.wpi.first.wpilibj.DigitalInput;

public class RecycleBinLift {
	private MotorState state;
	private boolean binInHand;

	public RecycleBinLift( MotorState state ) {
		this.state = state;
		binInHand = false;
	}

	public MotorState getMotorState() {
		return state;
	}

	public void checkLimit(){
		/*upper limit for the bin lift kept in robot*/
		if( upperLimit.get() ){	
			if( state == MotorState.UP )
				binInHand = true;
			state = MotorState.STOP;
		}
		/*lower limit for the bin lift kept in robot*/
		if( lowerLimit.get() ){
			if( state == MotorState.DOWN )
				binInHand = false;
		} 
	}

}