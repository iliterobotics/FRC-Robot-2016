package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.Sensor;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;

public class RecycleBinLift {
	//RECYCLE_BIN_LEFT_ENCODER, RECYCLE_BIN_RIGHT_ENCODER
	private MotorState state;
	private boolean hasBin;

	public RecycleBinLift( MotorState state ) {
		this.state = state;
		hasBin = false;
	}
	public MotorState getMotorState() {
		return state;
	}
	public boolean hasBin(){
		return hasBin;
	}
	public void updateLift(){
		if( state == MotorState.UP ){
			if( SensorInputControl.getInstance().getLimitSwitch( Sensor.RECYCLE_BIN_UPPER_LIMIT ).get() ){
				hasBin = true;
				state = MotorState.STOP;
			}
		}
		if( state == MotorState.DOWN ){
			if( SensorInputControl.getInstance().getLimitSwitch( Sensor.RECYCLE_BIN_LOWER_LIMIT ).get() ){
				hasBin = false;
				state = MotorState.STOP;
			}
		}	
	}
}