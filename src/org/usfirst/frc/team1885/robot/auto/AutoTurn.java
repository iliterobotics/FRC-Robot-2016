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
	
	private final static long steadyStatePeriod = 100;
	private long steadyStateTime;
	
	
	public AutoTurn(double inputAngle, double inputError) {
		angleControlLoop = new PID(0.012, 0.00005, 0);
		angle = inputAngle;
		relativeAngle = 0;
		error = inputError;
		steadyStateTime = 0;
	}
	public boolean execute() {
		rightAngleTraveled = SensorInputControl.getInstance().getNAVX().getYaw();
		if (Math.abs(rightAngleTraveled  - relativeAngle) <= error ) {
			
			this.steadyStateTime = System.currentTimeMillis();
			
			if(this.steadyStateTime > System.currentTimeMillis() + AutoTurn.steadyStatePeriod) {
				this.reset();
				return true;
			} else {
				return false;
			}
		} else {
			steadyStateTime = 0;
			rightDriveOutput = angleControlLoop.getPID(relativeAngle, rightAngleTraveled);
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
		relativeAngle = angle + SensorInputControl.getInstance().getNAVX().getYaw();
		reset();
		return true;
		
	}

}
