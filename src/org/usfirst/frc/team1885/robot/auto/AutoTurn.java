package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class AutoTurn extends AutoCommand{
	
	private PID angleControlLoop;
	private double angle;
	private double relativeAngle;
	private double error;
	private double rightAngleTraveled;
	private double leftDriveOutput;
	private double rightDriveOutput;
	
	private static final double MAX_TURN_SPEED = .75;
	
	private static final double MIN_TURN_SPEED = .15;
	
	private final static long steadyStatePeriod = 100;
	private long steadyStateStartTime;
	
	
	public AutoTurn(double inputAngle, double inputError) {
		angleControlLoop = new PID(0.01, 0.00003, 0);
		angle = inputAngle;
		relativeAngle = 0;
		error = inputError;
		steadyStateStartTime = 0;
	}
	public boolean execute() {
		rightAngleTraveled = SensorInputControl.getInstance().getNAVX().getYaw360();
		
		System.out.println(toString());
		double difference = (rightAngleTraveled - relativeAngle);
		
		if(Math.abs(difference) > Math.abs((360 - rightAngleTraveled) + relativeAngle)) {
			difference = ((360 - rightAngleTraveled) + relativeAngle);
		}
		
		if(Math.abs(difference) > Math.abs((360 - relativeAngle) + rightAngleTraveled)) {
			difference = -((360 - relativeAngle) + rightAngleTraveled);
		}
		
		
		
		if (Math.abs(difference) <= error ) {
			
			if(this.steadyStateStartTime == 0) {
				this.steadyStateStartTime = System.currentTimeMillis();
			}
						
			if(this.steadyStateStartTime + AutoTurn.steadyStatePeriod > System.currentTimeMillis()) {
				this.reset();
				return true;
			} else {
				return false;
			}
		} else {
			steadyStateStartTime = 0;
			rightDriveOutput = angleControlLoop.getPID(0, difference);
			
			if(rightDriveOutput > AutoTurn.MAX_TURN_SPEED) {
				rightDriveOutput = AutoTurn.MAX_TURN_SPEED;
			} else if(rightDriveOutput < -AutoTurn.MAX_TURN_SPEED) {
				rightDriveOutput = AutoTurn.MAX_TURN_SPEED;
			}
			
			leftDriveOutput = rightDriveOutput * -1;
			DrivetrainControl.getInstance().update(leftDriveOutput, rightDriveOutput);
			return false;
		}
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
		relativeAngle = (angle + SensorInputControl.getInstance().getNAVX().getYaw360()) % 360;
		reset();
		return true;
		
	}
	@Override
	public String toString() {
		return "AutoTurn [angleControlLoop=" + angleControlLoop + ", angle="
				+ angle + ", relativeAngle=" + relativeAngle + ", error="
				+ error + ", rightAngleTraveled=" + rightAngleTraveled
				+ ", leftDriveOutput=" + leftDriveOutput
				+ ", rightDriveOutput=" + rightDriveOutput
				+ ", steadyStateTime=" + steadyStateStartTime + "]";
	}

}
