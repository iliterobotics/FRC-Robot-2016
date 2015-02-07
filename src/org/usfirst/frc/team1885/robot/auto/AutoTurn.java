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
	
	private final static long steadyStatePeriod = 100;
	private long steadyStateTime;
	
	
	public AutoTurn(double inputAngle, double inputError) {
		angleControlLoop = new PID(0.012, 0.00005, 0);
		angle = inputAngle;
		error = inputError;
		SensorInputControl.getInstance().getNAVX().zeroYaw();
		steadyStateTime = 0;
	}
	public boolean execute() {
		rightAngleTraveled = SensorInputControl.getInstance().getNAVX().getYaw();
		if (Math.abs(rightAngleTraveled  - angle) <= error ) {
			
			this.steadyStateTime = System.currentTimeMillis();
			
			if(this.steadyStateTime > System.currentTimeMillis() + AutoTurn.steadyStatePeriod) {
				this.reset();
				return true;
			} else {
				return false;
			}
		} else {
			steadyStateTime = 0;
			rightDriveOutput = angleControlLoop.getPID(angle, rightAngleTraveled);
			leftDriveOutput = rightDriveOutput * -1;
			System.out.println("Autonomous::AutoTurn - [right, left, yaw]: " + rightDriveOutput + ", " + leftDriveOutput + ", " + rightAngleTraveled);
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
