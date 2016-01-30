package org.usfirst.frc.team1885.graveyard;


import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

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
	
	public AutoDriveForward(double sec, double pow) {
		rightDriveOutput = leftDriveOutput = -pow;
		
		distance = sec;
		DriverStation.reportError("Drive with " + pow + " power for " + sec + " seconds", false);
//		reset();
	}
	public boolean execute() {
	    DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveOutput);
	    DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveOutput);
	    updateOutputs();
	    Timer.delay(distance);
	    rightDriveOutput = leftDriveOutput = 0;
	    DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveOutput);
        DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveOutput);
	    updateOutputs();
	    return true;
	}
	public void reset() {
//		rightDistanceControlLoop.reset();
//		leftDistanceControlLoop.reset();
//		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).reset();
//		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).reset();
		DrivetrainControl.getInstance().setLeftDriveSpeed(0);
		DrivetrainControl.getInstance().setRightDriveSpeed(0);
	}
	
	public boolean updateOutputs() {
		RobotControlWithSRX.getInstance().updateDriveSpeed(DrivetrainControl.getInstance().getLeftDriveSpeed(), DrivetrainControl.getInstance().getRightDriveSpeed());
		return true;
	}
	
	public boolean init() {
		reset();
		return true;
		
	}
	
}
