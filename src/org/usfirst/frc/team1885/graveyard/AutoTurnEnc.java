package org.usfirst.frc.team1885.graveyard;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.config2015.RobotConfiguration;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

public class AutoTurnEnc extends AutoCommand{
	
	private PID angleControlLoop;
	private double angle;
	private double relativeAngle;
	private double error;
	private double leftAngleTraveled;
	private double rightAngleTraveled;
	private double leftDriveOutput;
	private double rightDriveOutput;
	
	private static final double MAX_TURN_SPEED = .75;
	
	private static final double MIN_TURN_SPEED = .25;
	
	private final static long steadyStatePeriod = 100;
	private long steadyStateStartTime;
	
	
	public AutoTurnEnc(double inputAngle, double inputError) {
		angleControlLoop = new PID(0.008, 0.00002, 0);
		angle = inputAngle;
		relativeAngle = 0;
		error = inputError;
		steadyStateStartTime = 0;
	}
	public boolean execute() {
		
		leftAngleTraveled = 360 * SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).getDistance() / RobotConfiguration.FRAME_LENGTH;
		rightAngleTraveled = 360 * SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).getDistance() / RobotConfiguration.FRAME_LENGTH;
		
		if (Math.abs(leftAngleTraveled  - angle) <= error && Math.abs(rightAngleTraveled  - angle) <= error ) {
			this.reset();
			return true;
		}
		
		System.out.println(toString());
		
//		rightDriveOutput = angleControlLoop.getPID(0, difference);
//		
//		if(rightDriveOutput > 0) {
//			rightDriveOutput = (rightDriveOutput < AutoTurnEnc.MIN_TURN_SPEED ? AutoTurnEnc.MIN_TURN_SPEED : rightDriveOutput);
//		} else if(rightDriveOutput < 0) {
//			rightDriveOutput = (rightDriveOutput > -AutoTurnEnc.MIN_TURN_SPEED ? -AutoTurnEnc.MIN_TURN_SPEED : rightDriveOutput);
//		}
//		
//		if(rightDriveOutput > AutoTurnEnc.MAX_TURN_SPEED) {
//			rightDriveOutput = AutoTurnEnc.MAX_TURN_SPEED;
//		} else if(rightDriveOutput < -AutoTurnEnc.MAX_TURN_SPEED) {
//			rightDriveOutput = -AutoTurnEnc.MAX_TURN_SPEED;
//		}
//		
//		leftDriveOutput = rightDriveOutput * -1;
//		DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveOutput);
//		DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveOutput);
		return false;
	}

	public void reset() {
		angleControlLoop.reset();
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
