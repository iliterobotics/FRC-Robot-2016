package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.lift.ToteLift;

public class AutoToteLift extends AutoCommand{
	private double height;
	private double error;
		
	public AutoToteLift(double h, double e) {
		height = h;
		error = e;
	}
	public boolean execute() {
		return ToteLift.getInstance().incrementLift(height, error);
	}

	public void reset() {
		ToteLift.getInstance().reset();	
		
	}
	
	public boolean updateOutputs() {
		ToteLift.getInstance().updateOutputs();
		return true;
	}

	public boolean init() {
		reset();
		ToteLift.getInstance().setScalingValue(height);
		return true;
	}

}
