package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class DriveForwardAuto implements AutoCommand{
	
	private PID distanceControlLoop;
	private double distance;
	private double error;
	private double leftDriveDistance;
	private double rightDriveDistance;
	private double leftDistanceTraveled;
	private double rightDistanceTraveled;
	public DriveForwardAuto(double d, double e) {
		distanceControlLoop = new PID();
		distance = d;
		error = e;
		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).reset();
		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).reset();
	}
	public boolean execute(double distance) {
		leftDistanceTraveled = SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).getDistance();
		rightDistanceTraveled = SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).getDistance();
		if (Math.abs(leftDistanceTraveled  - distance) <= error && Math.abs(rightDistanceTraveled  - distance) <= error ) {
			this.reset();
			return true;
		} else {
			leftDriveDistance = distanceControlLoop.getPID(distance, SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).getDistance());
			rightDriveDistance = distanceControlLoop.getPID(distance, SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).getDistance());
			RobotControl.getInstance().updateDriveSpeed(leftDriveDistance, rightDriveDistance);
			return false;
		}
	}
	public void reset() {
		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).reset();
		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).reset();
		RobotControl.getInstance().updateDriveSpeed(0, 0);
	}
}
