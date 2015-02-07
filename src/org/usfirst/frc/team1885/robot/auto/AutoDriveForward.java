package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class AutoDriveForward implements AutoCommand{
	
	private PID rightDistanceControlLoop;
	private PID leftDistanceControlLoop;
	private double distance;
	private double error;
	private double leftDriveOutput;
	private double rightDriveOutput; 
	private double leftDistanceTraveled;
	private double rightDistanceTraveled;
	public AutoDriveForward(double d, double e) {
		rightDistanceControlLoop = new PID(0.01, 0, 0);
		leftDistanceControlLoop = new PID(0.01, 0, 0);
		distance = d;
		error = e;
		reset();
	}
	public boolean execute() {
		System.out.println("AutoDriveFwd::[left dist, right dist] " + leftDistanceTraveled + ", " + rightDistanceTraveled);
		
		if (Math.abs(leftDistanceTraveled  - distance) <= error && Math.abs(rightDistanceTraveled  - distance) <= error ) {
			this.reset();
			return true;
		}
		
		if(Math.abs(leftDistanceTraveled  - distance) > error) {
			leftDistanceTraveled = SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).getDistance();
			leftDriveOutput = leftDistanceControlLoop.getPID(distance, leftDistanceTraveled);
		} else {
			leftDistanceControlLoop.reset();
			leftDriveOutput = 0;
		}
		
		if(Math.abs(rightDistanceTraveled  - distance) > error) {
			rightDistanceTraveled = -SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).getDistance();
			rightDriveOutput = rightDistanceControlLoop.getPID(distance, rightDistanceTraveled);
		} else {
			rightDistanceControlLoop.reset();
			rightDriveOutput = 0;
		}
		
		System.out.println("AutoDriveFwd::[left speed, right speed] " + leftDriveOutput + ", " + rightDriveOutput);
		
		RobotControl.getInstance().updateDriveSpeed(-leftDriveOutput, -rightDriveOutput);
		return false;
	}
	public void reset() {
		rightDistanceControlLoop.reset();
		leftDistanceControlLoop.reset();
		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).reset();
		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).reset();
		RobotControl.getInstance().updateDriveSpeed(0, 0);
	}
}
