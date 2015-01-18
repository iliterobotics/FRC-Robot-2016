package org.usfirst.frc.team1885.robot.modules.drivetrain;

import java.util.HashMap;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.DriveMode;
import org.usfirst.frc.team1885.robot.common.type.Sensor;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;


public class DrivetrainControl 
{
	/**
	 * drive mode where you can only move straight using the right joystick
	 */
	private double leftDriveSpeed;
	private double rightDriveSpeed;
	private DriveMode driveMode;
	private PID speedControlLoop;
	private final double maxSpeed;
	private final double diameter;
	private final double circumference;
	private  HashMap<Integer, Double> speeds;
	
	public DrivetrainControl(final double d, final double m) {
		maxSpeed = m;
		speeds = new HashMap<Integer, Double>();;
		diameter = d;
		circumference = 2 * Math.PI * (diameter/2);

		SensorInputControl.getInstance().getEncoder(Sensor.DRIVE_TRAIN_LEFT_ENCODER).setDistancePerPulse(circumference/360);
		
		driveMode = DriveMode.TANK;
	}
	public void addSpeed(Integer gear, Double speed) {
		speeds.put(gear, speed);
	}
	public Integer getTotes() {
		return 1; //CHANGE TO WHAT SENSOR INPUTS SAYS
	}
	public double getSpeed(double speed) {
		return speed * circumference; 
	}
	public double getDistance () {
		return SensorInputControl.getInstance().getEncoderTicks(Sensor.DRIVE_TRAIN_LEFT_ENCODER) * circumference;
	}
	public void update(double leftJoystick, double rightJoystick) {
		
		
		
		leftDriveSpeed = leftJoystick * (speeds.get(getTotes()) / maxSpeed);
		rightDriveSpeed = rightJoystick * (speeds.get(getTotes()) / maxSpeed);
		
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
		return rightDriveSpeed * (speeds.get(getTotes()) / maxSpeed);
	}
	/**
	 * @param rightDriveSpeed the rightDriveSpeed to set
	 */
	public void setRightDriveSpeed(double rightDriveSpeed) {
		this.rightDriveSpeed = rightDriveSpeed * (speeds.get(getTotes()) / maxSpeed);;
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
		this.rightDriveSpeed = driveSpeed * (speeds.get(getTotes()) / maxSpeed);;
		this.leftDriveSpeed = driveSpeed * (speeds.get(getTotes()) / maxSpeed);;
	}
}
