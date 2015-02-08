package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.modules.lift.ToteLift;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class AutoToteLift implements AutoCommand{
	private PID heightControlLoop;
	private double height;
	private double error;
	private double heightOutput;
	private double heightTraveled;
	
	public AutoToteLift(double h, double e) {
		heightControlLoop = new PID(0.08, 0.0001, 0);
		height = h;
		error = e;
		SensorInputControl.getInstance().getEncoder(SensorType.TOTE_ENCODER).reset();
	}
	public boolean execute() {
		heightTraveled = SensorInputControl.getInstance().getEncoder(SensorType.TOTE_ENCODER).getDistance();
		if (Math.abs(heightTraveled  - height) <= error ) {
			this.reset(); 
			return true;
		} else {
			heightOutput = heightControlLoop.getPID(height, SensorInputControl.getInstance().getEncoder(SensorType.TOTE_ENCODER).getDistance());
			ToteLift.getInstance().updateLift(heightOutput);
			RobotControl.getInstance().updateToteMotor(ToteLift.getInstance().getSpeed());
			return false;
		}
	}

	public void reset() {
		heightControlLoop.reset();
		SensorInputControl.getInstance().getEncoder(SensorType.TOTE_ENCODER).reset();
		ToteLift.getInstance().updateLift(0);
		RobotControl.getInstance().updateToteMotor(0);		
		
	}

}
