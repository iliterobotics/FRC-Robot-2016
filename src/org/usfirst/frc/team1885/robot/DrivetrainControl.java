package org.usfirst.frc.team1885.robot;


public class DrivetrainControl 
{
	private double leftDriveSpeed;
	private double rightDriveSpeed;
	public DrivetrainControl()
	{
		leftDriveSpeed = 0;
		rightDriveSpeed = 0; 
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
	
	
}
