package org.usfirst.frc.team1885.robot.output;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.hal.CanTalonSRX;

public class RobotControlWithSRX 
{
	public static RobotControlWithSRX instance;
	private List<CANTalon> leftDrive;
	private List<CANTalon> rightDrive;
	private Map<RobotMotorType, CANTalon> talons;
	public static synchronized RobotControlWithSRX getInstance() {
		if (instance == null) {
			instance = new RobotControlWithSRX();
		}
		return instance;
	}
	protected RobotControlWithSRX()
	{
		leftDrive = new ArrayList<CANTalon>();
		rightDrive = new ArrayList<CANTalon>();
	}
	public void addTalonOutput(RobotMotorType type, int port) {
		if (type == RobotMotorType.LEFT_DRIVE) {
			leftDrive.add(new CANTalon(port));
		} 
		else if (type == RobotMotorType.RIGHT_DRIVE) {
			// add to right motor
			rightDrive.add(new CANTalon(port));
		}
		else
		{
		    talons.put(type, new CANTalon(port));
		}
	}
	public void updateDriveSpeed(double leftspeed, double rightspeed) {
		for (CANTalon leftMotor : leftDrive) {
			leftMotor.set(leftspeed);
			//System.out.println(leftMotor.getOutputVoltage() + "Voltage");
		}
		for (CANTalon rightMotor : rightDrive) {
			rightMotor.set(-rightspeed);
		}
	}
	public List<CANTalon> getLeftDrive()
	{
	    return leftDrive;
	}
	public List<CANTalon> getRightDrive()
	{
	    return rightDrive;
	}
	public Map<RobotMotorType, CANTalon> getTalons()
	{
	    return this.talons;
	}
}
