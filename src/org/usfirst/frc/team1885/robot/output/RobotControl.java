package org.usfirst.frc.team1885.robot.output;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class RobotControl 
{
	
	private List<Talon> leftDrive;
	private List<Talon> rightDrive;
	private Map<RobotMotorType, Talon> outputTalons;
	private Map<RobotPneumaticType, Solenoid> outputSolenoids;
	/*
	private Talon leftDrive1;
	private Talon leftDrive2;
	private Talon rightDrive1;
	private Talon rightDrive2;
	private Talon toteMotor;
	private Talon recycleBinMotor;
	
	//TODO: convert to solenoid
	private Solenoid grabberPneumatic;
	private Solenoid leftShifterPneumatic;
	private Solenoid rightShifterPneumatic;
	*/
	public RobotControl()
	{
		
		outputSolenoids = new HashMap<RobotPneumaticType, Solenoid>();
		outputTalons = new HashMap<RobotMotorType,Talon>();
		//leftDrive1 = new Talon(0);		
		addTalonOutput(RobotMotorType.LEFT_DRIVE, 0);
		//leftDrive2 = new Talon(1);
		addTalonOutput(RobotMotorType.LEFT_DRIVE, 1);
		//rightDrive1 = new Talon(2);
		addTalonOutput(RobotMotorType.RIGHT_DRIVE, 2);
		//rightDrive2 = new Talon(3);
		addTalonOutput(RobotMotorType.RIGHT_DRIVE, 3);
		//grabberPneumatic = new Solenoid(7);		
		addPneumaticOutput(RobotPneumaticType.GRABBER_PNEUMATIC, 7);
		//leftShifterPneumatic = new Solenoid(8);
		addPneumaticOutput(RobotPneumaticType.LEFT_SHIFTER_PNEUMATIC, 8);
		//rightShifterPneumatic = new Solenoid(9);
		addPneumaticOutput(RobotPneumaticType.RIGHT_SHIFTER_PNEUMATIC, 9);
		//toteMotor = new Talon(4);
		addTalonOutput(RobotMotorType.TOTE_LIFT, 4);
		//recycleBinMotor = new Talon(6);
		addTalonOutput(RobotMotorType.RECYCLE_LIFT, 4);
	}
	
	public void addTalonOutput(RobotMotorType type, int port) {
		if(type == RobotMotorType.LEFT_DRIVE) {
			leftDrive.add(new Talon(port));
		} else if(type == RobotMotorType.RIGHT_DRIVE) {
			//add to right motor
			rightDrive.add(new Talon(port));
		}
		else {
			outputTalons.put(type, new Talon(port));
		}
	}
	
	public void addPneumaticOutput(RobotPneumaticType type, int port)
	{		
		outputSolenoids.put(type,new Solenoid(port));		
	}
	public void updateDriveSpeed(double leftspeed, double rightspeed)
	{
		for(Talon leftMotor : leftDrive)
		{
			leftMotor.set(-leftspeed);
		}
		for(Talon rightMotor : rightDrive)
		{
			rightMotor.set(rightspeed);
		}		
	}
	public void updateGrabberPneumatics( boolean start )
	{
		outputSolenoids.get(RobotPneumaticType.GRABBER_PNEUMATIC).set(start);
	}
	public void updateLeftShifter( boolean start )
	{	
		outputSolenoids.get(RobotPneumaticType.LEFT_SHIFTER_PNEUMATIC).set(start);	
	}
	
	public void updateRightShifter( boolean start )
	{
		outputSolenoids.get(RobotPneumaticType.RIGHT_SHIFTER_PNEUMATIC).set(start);	
	}
	public void updateToteMotor(double speed)
	{
		outputTalons.get(RobotMotorType.TOTE_LIFT).set(speed);	
	}
	public void updateRecycleMotor(double speed)
	{
		outputTalons.get(RobotMotorType.RECYCLE_LIFT).set(speed);
	}
}
