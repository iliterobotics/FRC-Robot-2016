package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class AutoArc implements AutoCommand{
	
	private double threshold;
	
	private AutoTurn autoTurn;
	private AutoDriveForward autoDriveForward;
	
	public AutoArc(double inputDistance, double inputDistanceError, double inputAngle, double inputAngleError, double inputThreshold) {

		autoDriveForward = new AutoDriveForward(inputDistance, inputDistanceError);
		autoTurn = new AutoTurn(inputAngle, inputAngleError);
		
		this.threshold = inputThreshold;
		
		reset();
	}
	public boolean execute() {
		
		//execute drive forward
		boolean driveForwardState = autoDriveForward.execute();
		
		//get the left/right values from DriveTrainControl
		double fwdLeftOutput = DrivetrainControl.getInstance().getLeftDriveSpeed();
		double fwdRightOutput = DrivetrainControl.getInstance().getRightDriveSpeed();
		
		//execute turn
		boolean turnState = autoTurn.execute();
		
		//get the left/right outputs
		double turnLeftOutput = DrivetrainControl.getInstance().getLeftDriveSpeed();
		
		//set new output
		if(turnLeftOutput > threshold) {
			turnLeftOutput = threshold;
		} else if(turnLeftOutput < -threshold) {
			turnLeftOutput = -threshold;
		}
			
		DrivetrainControl.getInstance().update(fwdLeftOutput - turnLeftOutput, fwdRightOutput + turnLeftOutput);
				
		return (driveForwardState && turnState);
	}
	public void reset() {
		autoTurn.reset();
		autoDriveForward.reset();
		RobotControl.getInstance().updateDriveSpeed(0, 0);
	}
	
	public boolean updateOutputs() {
		RobotControl.getInstance().updateDriveSpeed(DrivetrainControl.getInstance().getLeftDriveSpeed(), DrivetrainControl.getInstance().getRightDriveSpeed());
		return true;
	}
	
	public void init() {
		reset();	
	}
}