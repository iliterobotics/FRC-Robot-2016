package org.usfirst.frc.team1885.robot.manipulator;

import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;

public class ClawControl 
{
	private ClawControl instance;
	private boolean isIncrementing;
	
	protected ClawControl(){
	    isIncrementing = false;
	}
	public ClawControl getInstance(){
	    if(instance == null){
	        instance = new ClawControl();
	    }
	    return instance;
	}
	public void updateClaw(){
	    if(DriverInputControl.getInstance().getButton(
                RobotButtonType.TURN_WRIST) && !isIncrementing){
	        this.isIncrementing = incrementClaw();
	    }
	}
	public boolean incrementClaw(){
	    if(!SensorInputControl.getInstance().isActive(SensorType.LINE_SENSOR)){
	        return true;
	    }
	    return false;
	}
	public boolean clawState(){
	    return isIncrementing;
	}
}
