package org.usfirst.frc.team1885.robot.modules.drivetrain;

import java.util.HashMap;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.DriveMode;
import org.usfirst.frc.team1885.robot.common.type.GearState;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class DrivetrainControl 
{
	/**
	 * drive mode where you can only move straight using the right joystick
	 */
	private double leftDriveSpeed;
	private double rightDriveSpeed;
	private DriveMode driveMode;
	private GearState gearState;
	private PID speedControlLoop;
	private final double maxSpeed;
	private final double diameter;
	private final double circumference;
	private  HashMap<Integer, Double> speeds;
	private DriverInputControl driverInput;
	private static DrivetrainControl instance;
	
	protected DrivetrainControl(final double d, final double m) {
		maxSpeed = m;
		speeds = new HashMap<Integer, Double>();
		diameter = d;
		circumference = Math.PI * (diameter);

		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).setDistancePerPulse(circumference/256);
		SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).setDistancePerPulse(circumference/256);
		driveMode = DriveMode.TANK;
		setGearState(GearState.HIGH_GEAR);
		driverInput = DriverInputControl.getInstance();
	}
	public static DrivetrainControl getInstance(){
	    if(instance == null){
	        instance = new DrivetrainControl(4.0, 15.0);
	    }
	    return instance;
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
		return SensorInputControl.getInstance().getEncoderTicks(SensorType.DRIVE_TRAIN_LEFT_ENCODER) * circumference;
	}
	
	public void update()
	{
		if (driverInput.getButton(RobotButtonType.GEAR_SHIFT)){
			setGearState(GearState.LOW_GEAR);
		}
		else{
			setGearState(GearState.HIGH_GEAR);
		}
		
		//FIXME: add slow straight drive state + button
		
		update( driverInput.getLeftDrive(), driverInput.getRightDrive() );
	}
	
	public void update(double leftJoystick, double rightJoystick) {
		if(Math.abs(leftDriveSpeed - rightDriveSpeed) < DriverInputControl.DEADZONE) {
			leftDriveSpeed = rightDriveSpeed = (leftDriveSpeed + rightDriveSpeed) / 2;
		} else {
			leftDriveSpeed = leftJoystick * (speeds.get(getTotes()) / maxSpeed);
			rightDriveSpeed = rightJoystick * (speeds.get(getTotes()) / maxSpeed);
		}
		
		leftDriveSpeed = DriverInputControl.expScale(leftDriveSpeed);
		rightDriveSpeed = DriverInputControl.expScale(rightDriveSpeed);
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
	public GearState getGearState() {
		return gearState;
	}
	public boolean getGearValue() {
    	if(this.gearState == GearState.HIGH_GEAR) {
    		return true;
    	} else {
    		return false;
    	}
    }
	public void setGearState(GearState gearState) {
		this.gearState = gearState;
	}
	
	public void updateOutputs() {
		RobotControl.getInstance().updateShifter(getGearValue());
		RobotControl.getInstance().updateDriveSpeed(leftDriveSpeed, rightDriveSpeed);
	}
}
