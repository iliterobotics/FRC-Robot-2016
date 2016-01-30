package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.graveyard.RobotControl;
import org.usfirst.frc.team1885.graveyard.SensorInputControl;
import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

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
	private double prevErrorLeft = 0;
	private double prevErrorRight = 0;
	private double timeout = 250;
	private double stallStartTime = 0;
	
	private static double MIN_SPEED = 0.0;
	
	public AutoDriveForward(double d, double e, int n) {
		rightDistanceControlLoop = new PID(.35, 0.002, 0);
		rightDistanceControlLoop.setScalingValue(d);
		leftDistanceControlLoop = new PID(.35, 0.002, 0);
		leftDistanceControlLoop.setScalingValue(d);
		numberOfEncoders = n;
		distance = d;
		error = e;
		reset();
	}
	public boolean execute() {
//		System.out.println("AutoDriveFwd::[left dist, right dist] " + leftDistanceTraveled + ", " + rightDistanceTraveled);
				
		if( numberOfEncoders == 1) {
			if (Math.abs(rightDistanceTraveled  - distance) <= error || Math.abs(leftDistanceTraveled  - distance) <= error) {
				this.reset();
				return true;
			}
		}
		else if( numberOfEncoders == 2) {
			if (Math.abs(leftDistanceTraveled  - distance) <= error && Math.abs(rightDistanceTraveled  - distance) <= error ) {
				this.reset();
				return true;
			}
			
//			if(Math.abs(prevErrorLeft - leftDistanceTraveled) <= 3 && Math.abs(prevErrorRight - rightDistanceTraveled) <= 3 && stallStartTime == 0) {
//				stallStartTime = System.currentTimeMillis();
//			} else {
//				stallStartTime = 0;
//			}
		}
//		System.out.println(stallStartTime + " " + Math.abs(prevErrorLeft - leftDistanceTraveled));
//		if(stallStartTime + timeout > System.currentTimeMillis()) {
//			this.reset();
//			return true;
//		}
		
		if(Math.abs(leftDistanceTraveled  - distance) > error) {
			leftDistanceTraveled = SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).getDistance();
			leftDriveOutput = leftDistanceControlLoop.getPID(distance, leftDistanceTraveled);
			
			if(leftDriveOutput > 0) {
				leftDriveOutput = (leftDriveOutput < AutoDriveForward.MIN_SPEED ? AutoDriveForward.MIN_SPEED : leftDriveOutput);
			} else if(leftDriveOutput < 0) {
				leftDriveOutput = (leftDriveOutput > -AutoDriveForward.MIN_SPEED ? -AutoDriveForward.MIN_SPEED : leftDriveOutput);
			}
		} else {
			leftDistanceControlLoop.reset();
			leftDriveOutput = 0;
		}
		
		if(Math.abs(rightDistanceTraveled  - distance) > error) {
			rightDistanceTraveled = -SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).getDistance();
			rightDriveOutput = rightDistanceControlLoop.getPID(distance, rightDistanceTraveled);
			
			if(rightDriveOutput > 0) {
				rightDriveOutput = (rightDriveOutput < AutoDriveForward.MIN_SPEED ? AutoDriveForward.MIN_SPEED : rightDriveOutput);
			} else if (rightDriveOutput < 0) {
				rightDriveOutput = (rightDriveOutput > -AutoDriveForward.MIN_SPEED ? -AutoDriveForward.MIN_SPEED : rightDriveOutput);
			}
		} else {
			rightDistanceControlLoop.reset();
			rightDriveOutput = 0;
		}
		
//		System.out.println("AutoDriveFwd::[left speed, right speed] " + leftDriveOutput + ", " + rightDriveOutput);
		
		DrivetrainControl.getInstance().setLeftDriveSpeed(-leftDriveOutput);
		DrivetrainControl.getInstance().setRightDriveSpeed(-rightDriveOutput);
		
		prevErrorLeft = leftDistanceTraveled - distance;
		prevErrorRight = rightDistanceTraveled - distance;

		
		return false;
		
	}
	public void reset() {
		rightDistanceControlLoop.reset();
		leftDistanceControlLoop.reset();
		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).reset();
		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).reset();
		DrivetrainControl.getInstance().setLeftDriveSpeed(0);
		DrivetrainControl.getInstance().setRightDriveSpeed(0);
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
