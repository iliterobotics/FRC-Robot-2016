package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class AutoDriveForward extends AutoCommand{
	
	private PID rightDistanceControlLoop;
	private PID leftDistanceControlLoop;
	private double distance;
	private double error;
	private int numberOfEncoders;
	private double leftDriveOutput;
	private double rightDriveOutput; 
	private double leftDistanceTraveled;
	private double rightDistanceTraveled;
	public AutoDriveForward(double d, double e, int n) {
		rightDistanceControlLoop = new PID(0.01, 0.00001, 0);
		leftDistanceControlLoop = new PID(0.01, 0.00001, 0);
		numberOfEncoders = n;
		distance = d;
		error = e;
		reset();
	}
	public boolean execute() {
		System.out.println("AutoDriveFwd::[left dist, right dist] " + leftDistanceTraveled + ", " + rightDistanceTraveled);
		if( numberOfEncoders == 1) {
			if (Math.abs(rightDistanceTraveled  - distance) <= error ) {
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
			
			DrivetrainControl.getInstance().update(-leftDriveOutput, -rightDriveOutput);
			
			return false;
		}
		else if( numberOfEncoders == 2) {
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
			
			DrivetrainControl.getInstance().update(-leftDriveOutput, -rightDriveOutput);
			
			return false;
		}
		else {
			return false;
		}
		
	}
	public void reset() {
		rightDistanceControlLoop.reset();
		leftDistanceControlLoop.reset();
		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).reset();
		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).reset();
		RobotControl.getInstance().updateDriveSpeed(0, 0);
	}
	
	public boolean updateOutputs() {
		RobotControl.getInstance().updateDriveSpeed(DrivetrainControl.getInstance().getLeftDriveSpeed(), DrivetrainControl.getInstance().getRightDriveSpeed());
		return true;
	}
	
	public boolean init() {
		reset();
		return true;
		
	}
	
}
