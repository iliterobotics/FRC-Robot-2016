package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AutoTurn extends AutoCommand{
	
	private PID angleControlLoop;
	private double angle;
	private double relativeAngle;
	private double error;
	private double rightAngleTraveled;
	private double leftDriveOutput;
	private double rightDriveOutput;
	
	private static final double MAX_TURN_SPEED = 1.0;
	
//	private static final double MIN_TURN_SPEED = .25;
	
	private final static long steadyStatePeriod = 100;
	private long steadyStateStartTime;
	
	
	public AutoTurn(double inputAngle, double inputError) {
		angleControlLoop = new PID(.6, 0.0015, 0);
		angleControlLoop.setScalingValue(inputAngle);
		angle = inputAngle;
		relativeAngle = 0;
		error = inputError;
		steadyStateStartTime = 0;
	}
	public boolean execute() {
	    //TODO adjust to match current set up
//		rightAngleTraveled = SensorInputControl.getInstance().getNAVX().getYaw360();
		
		
		double difference = (rightAngleTraveled - relativeAngle);
		
		if(Math.abs(difference) > Math.abs((360 - rightAngleTraveled) + relativeAngle)) {
			difference = -((360 - rightAngleTraveled) + relativeAngle);
		}
		
		if(Math.abs(difference) > Math.abs((360 - relativeAngle) + rightAngleTraveled)) {
			difference = ((360 - relativeAngle) + rightAngleTraveled);
		}
		
//		System.out.println(toString());
//		System.out.println(difference);
		
		if (Math.abs(difference) <= error ) {
			
			if(this.steadyStateStartTime == 0) {
				this.steadyStateStartTime = System.currentTimeMillis();
			}
						
			if(System.currentTimeMillis() > this.steadyStateStartTime + AutoTurn.steadyStatePeriod) {
				this.reset();
				return true;
			} else {
				return false;
			}
		} else {
			steadyStateStartTime = 0;
		}
		
		rightDriveOutput = angleControlLoop.getPID(0, difference);
		
//		if(rightDriveOutput > 0) {
//			rightDriveOutput = (rightDriveOutput < AutoTurn.MIN_TURN_SPEED ? AutoTurn.MIN_TURN_SPEED : rightDriveOutput);
//		} else if(rightDriveOutput < 0) {
//			rightDriveOutput = (rightDriveOutput > -AutoTurn.MIN_TURN_SPEED ? -AutoTurn.MIN_TURN_SPEED : rightDriveOutput);
//		}
		
		if(rightDriveOutput > AutoTurn.MAX_TURN_SPEED) {
			rightDriveOutput = AutoTurn.MAX_TURN_SPEED;
		} else if(rightDriveOutput < -AutoTurn.MAX_TURN_SPEED) {
			rightDriveOutput = -AutoTurn.MAX_TURN_SPEED;
		}
		 
		System.out.println("AutoTurn::execute [difference, power] - " + difference + ", " + rightDriveOutput);
		
		leftDriveOutput = rightDriveOutput * -1;
		DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveOutput);
		DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveOutput);
		return false;
	}

	public void reset() {
		angleControlLoop.reset();
		RobotControlWithSRX.getInstance().updateDriveSpeed(0, 0);
	}
	
	public boolean updateOutputs() {
		RobotControlWithSRX.getInstance().updateDriveSpeed(DrivetrainControl.getInstance().getLeftDriveSpeed(), DrivetrainControl.getInstance().getRightDriveSpeed());
		return true;
	}
	
	public boolean init() {
//		relativeAngle = (angle + SensorInputControl.getInstance().getNAVX().getYaw360());
		
		relativeAngle = (relativeAngle < 0 ? 360 + relativeAngle : relativeAngle % 360);
		
		reset();
		return true;
		
	}
	@Override
	public String toString() {
		return "AutoTurn [angle="
				+ angle + ", relativeAngle=" + relativeAngle + ", error="
				+ error + ", rightAngleTraveled=" + rightAngleTraveled
				+ ", leftDriveOutput=" + leftDriveOutput
				+ ", rightDriveOutput=" + rightDriveOutput
				+ ", steadyStateTime=" + steadyStateStartTime + "]";
	}

}
