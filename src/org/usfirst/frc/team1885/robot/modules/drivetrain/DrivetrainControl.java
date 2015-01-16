package org.usfirst.frc.team1885.robot.modules.drivetrain;

import org.usfirst.frc.team1885.robot.common.type.DriveMode;


public class DrivetrainControl 
{
	/**
	 * drive mode where you can only move straight using the right joystick
	 */
	private double leftDriveSpeed;
	private double rightDriveSpeed;
	private DriveMode driveMode;
	public DrivetrainControl()
	{
		leftDriveSpeed = 0;
		rightDriveSpeed = 0; 
		driveMode = DriveMode.TANK;
	}
	public void update(double leftJoystick, double rightJoystick)
	{
		//calculate
		leftDriveSpeed = leftJoystick;
		rightDriveSpeed = rightJoystick;
	}
	/**
	 * @return the leftDriveSpeed
	 */
	public double getLeftDriveSpeed() {
		return leftDriveSpeed;
	}
	/**
	 * @param leftDriveSpeed the leftDriveSpeed to set
	 */
	public void setLeftDriveSpeed(double leftDriveSpeed) {
			this.leftDriveSpeed = leftDriveSpeed;
	}
	/**
	 * @return the rightDriveSpeed
	 */
	public double getRightDriveSpeed() {
		return rightDriveSpeed;
	}
	/**
	 * @param rightDriveSpeed the rightDriveSpeed to set
	 */
	public void setRightDriveSpeed(double rightDriveSpeed) {
		this.rightDriveSpeed = rightDriveSpeed;
	}
	/**
	 * @return the driveMode
	 */
	public DriveMode getDriveMode() {
		return driveMode;
	}
	/**
	 * changes driveMode from TANK drive to STRAIGHT drive
	 * 		STRAIGHT only uses the right joystick to drive straight
	 */
	public void toggleDriveMode() {
		if( driveMode == DriveMode.TANK ) {
			driveMode = DriveMode.STRAIGHT;
		}
		else {
			driveMode = DriveMode.TANK;
		}
	}
	/**
	 * @param driveSpeed sets both drive speeds to a single speed controlled by the 
	 * right joystick
	 */
	public void straightDrive(double driveSpeed) {
		this.rightDriveSpeed = driveSpeed;
		this.leftDriveSpeed = driveSpeed;
	}
}