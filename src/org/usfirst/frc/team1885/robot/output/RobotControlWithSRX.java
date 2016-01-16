package org.usfirst.frc.team1885.robot.output;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;

import edu.wpi.first.wpilibj.Talon;

public class RobotControlWithSRX 
{
	public static RobotControlWithSRX instance;
	private List<Talon> leftDrive;
	private List<Talon> rightDrive;
	private Map<RobotMotorType, Talon> talons;
	public static synchronized RobotControlWithSRX getInstance() {
		if (instance == null) {
			instance = new RobotControlWithSRX();
		}
		return instance;
	}
	protected RobotControlWithSRX()
	{
		leftDrive = new ArrayList<Talon>();
		rightDrive = new ArrayList<Talon>();
	}
	public void addTalonOutput(RobotMotorType type, int port) {
		if (type == RobotMotorType.LEFT_DRIVE) {
			leftDrive.add(new Talon(port));
		} 
		else if (type == RobotMotorType.RIGHT_DRIVE) {
			rightDrive.add(new Talon(port));
		}
		else
		{
		    talons.put(type, new Talon(port));
		}
	}
	public void updateDriveSpeed(double leftspeed, double rightspeed) {
		for (Talon leftMotor : leftDrive) {
			leftMotor.set(leftspeed);
			//System.out.println(leftMotor.getOutputVoltage() + "Voltage");
		}
		for (Talon rightMotor : rightDrive) {
			rightMotor.set(-rightspeed);
		}
	}
	public List<Talon> getLeftDrive()
	{
	    return leftDrive;
	}
	public List<Talon> getRightDrive()
	{
	    return rightDrive;
	}
	public Map<RobotMotorType, Talon> getTalons()
	{
	    return this.talons;
	}
}
