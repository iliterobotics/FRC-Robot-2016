package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.modules.lift.RecycleBinLift;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class AutoRecyleLift implements AutoCommand{
	private PID heightControlLoop;
	private double height;
	private double error;
	private double heightOutput;
	private double heightTraveled;
	
	public AutoRecyleLift(double h, double e) {
		heightControlLoop = new PID();
		height = h;
		error = e;
		SensorInputControl.getInstance().getEncoder(SensorType.RECYCLE_BIN_ENCODER).reset();
	}
	public boolean execute() {
		heightTraveled = SensorInputControl.getInstance().getEncoder(SensorType.RECYCLE_BIN_ENCODER).getDistance();
		if (Math.abs(heightTraveled  - height) <= error ) {
			this.reset();
			return true;
		} else {
			heightOutput = heightControlLoop.getPID(height, SensorInputControl.getInstance().getEncoder(SensorType.RECYCLE_BIN_ENCODER).getDistance());
			RecycleBinLift.getInstance().updateLift(heightOutput);
			RobotControl.getInstance().updateRecycleMotor(RecycleBinLift.getInstance().getSpeed());
			return false;
		}
	}

	public void reset() {
		heightControlLoop.reset();
		SensorInputControl.getInstance().getEncoder(SensorType.RECYCLE_BIN_ENCODER).reset();
		RecycleBinLift.getInstance().updateLift(0);
		RobotControl.getInstance().updateRecycleMotor(0);		
		
	}

}
