package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class AutoDriveForwardLidar implements AutoCommand{
	
	private PID distanceControlLoop;
	private double stopDistance;
	private double error;
	private double driveOutput; 
	private double distanceFromTarget;
	public AutoDriveForwardLidar(double d, double e) {
		distanceControlLoop = new PID(0.01, 0.00001, 0);
		stopDistance = d;
		error = e;
		reset();
	}
	public boolean execute() {
		System.out.println("AutoDriveFwd::[dist] " + distanceFromTarget);
		
		if (Math.abs(distanceFromTarget  - stopDistance) <= error) {
			this.reset();
			return true;
		}
		
		if(Math.abs(distanceFromTarget  - stopDistance) > error) {
			distanceFromTarget = SensorInputControl.getInstance().getLidarSensor(SensorType.LIDAR).getDistance();
			driveOutput = distanceControlLoop.getPID(stopDistance, distanceFromTarget);
		} else {
			distanceControlLoop.reset();
			driveOutput = 0;
		}
		
		System.out.println("AutoDriveFwd::[left speed, right speed] " + driveOutput);
		
		RobotControl.getInstance().updateDriveSpeed(-driveOutput, -driveOutput);
		return false;
	}
	public void reset() {
		distanceControlLoop.reset();
		RobotControl.getInstance().updateDriveSpeed(0, 0);
	}
}
