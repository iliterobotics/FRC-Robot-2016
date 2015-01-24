package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class AutoTurn implements AutoCommand{
	
	private PID angleControlLoop;
	private double angle;
	private double error;
	private double rightAngleTraveled;
	private double leftDriveOutput;
	private double rightDriveOutput;
	
	
	public AutoTurn(double inputAngle, double inputError) {
		angleControlLoop = new PID();
		angle = inputAngle;
		error = inputError;
		SensorInputControl.getInstance().getNAVX().zeroYaw();
	}
	public boolean execute() {
		rightAngleTraveled = SensorInputControl.getInstance().getNAVX().getYaw();
		if (Math.abs(rightAngleTraveled  - angle) <= error ) {
			this.reset();
			return true;
		} else {
			rightDriveOutput = angleControlLoop.getPID(angle, SensorInputControl.getInstance().getNAVX().getYaw());
			leftDriveOutput = rightDriveOutput * -1;
			RobotControl.getInstance().updateDriveSpeed(leftDriveOutput, rightDriveOutput);
			return false;
		}
	}

	public void reset() {
		angleControlLoop.reset();
		RobotControl.getInstance().updateDriveSpeed(0, 0);
		SensorInputControl.getInstance().getNAVX().zeroYaw();
	}

}
