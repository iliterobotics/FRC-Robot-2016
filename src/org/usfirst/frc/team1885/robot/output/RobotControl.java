package org.usfirst.frc.team1885.robot.output;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Talon;

public class RobotControl 
{
	private Talon leftDrive1;
	private Talon leftDrive2;
	private Talon rightDrive1;
	private Talon rightDrive2;
	private Talon toteMotor;
	private Talon recycleBinMotor;
	private Compressor grabberPneumatic;
	private Compressor leftShifterPneumatic;
	private Compressor rightShifterPneumatic;
	public RobotControl()
	{
		leftDrive1 = new Talon(0);
		leftDrive2 = new Talon(1);
		rightDrive1 = new Talon(2);
		rightDrive2 = new Talon(3);
		grabberPneumatic = new Compressor(7);
		leftShifterPneumatic = new Compressor(8);
		rightShifterPneumatic = new Compressor(9);
		toteMotor = new Talon(4);
		recycleBinMotor = new Talon(5);
	}
	
	public void updateDriveSpeed(double leftspeed, double rightspeed)
	{
		leftDrive1.set(-leftspeed);
		leftDrive2.set(-leftspeed);
		rightDrive1.set(rightspeed);
		rightDrive2.set(rightspeed);		
	}
	public void updateGrabberPneumatics( boolean start )
	{
		if(start)
		{
			grabberPneumatic.start();
		}
		else
		{
			grabberPneumatic.stop();
		}
	}
	public void updateLeftShifter( boolean start )
	{
		if(start)
		{
			leftShifterPneumatic.start();
		}
		else
		{
			leftShifterPneumatic.stop();
		}
	}
	
	public void updateRightShifter( boolean start )
	{
		if(start)
		{
			rightShifterPneumatic.start();
		}
		else
		{
			rightShifterPneumatic.stop();
		}
	}
	public void updateToteMotor(double speed)
	{
		toteMotor.set(speed);	
	}
	public void updateRecycleMotor(double speed)
	{
		recycleBinMotor.set(speed);
	}
}
