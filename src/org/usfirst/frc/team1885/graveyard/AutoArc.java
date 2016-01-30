package org.usfirst.frc.team1885.graveyard;

import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

public class AutoArc extends AutoCommand{
	
	private double turnPower;
	
	private AutoDriveForward autoDriveForward;
	
	boolean driveForwardState;
	
	public AutoArc(double inputDistance, double inputDistanceError, double turnPower) {

		autoDriveForward = new AutoDriveForward(inputDistance, inputDistanceError);
		
		this.turnPower = turnPower;
		
		reset();
	}
	
	
	public boolean execute() {
		
		//execute drive forward
		if(!driveForwardState) {
			driveForwardState = autoDriveForward.execute();
		}
		
		//get the left/right values from DriveTrainControl
		double fwdLeftOutput = DrivetrainControl.getInstance().getLeftDriveSpeed();
		double fwdRightOutput = DrivetrainControl.getInstance().getRightDriveSpeed();
			
		DrivetrainControl.getInstance().update(fwdLeftOutput + this.turnPower, fwdRightOutput);
		
		return driveForwardState;
	}
	public void reset() {
		driveForwardState = false;
		autoDriveForward.reset();
		RobotControl.getInstance().updateDriveSpeed(0, 0);
	}
	
	public boolean updateOutputs() {
		RobotControl.getInstance().updateDriveSpeed(DrivetrainControl.getInstance().getLeftDriveSpeed(), DrivetrainControl.getInstance().getRightDriveSpeed());
		return true;
	}
	
	public boolean init() {
		
		reset();	
		return (autoDriveForward.init());
	}
}