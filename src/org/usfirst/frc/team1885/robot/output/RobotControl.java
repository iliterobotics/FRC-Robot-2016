package org.usfirst.frc.team1885.robot.output;

import edu.wpi.first.wpilibj.Talon;

public class RobotControl 
{
	private Talon leftDrive1;
	private Talon leftDrive2;
	private Talon rightDrive1;
	private Talon rightDrive2;
	public RobotControl()
	{
		leftDrive1 = new Talon(0);
		leftDrive2 = new Talon(1);
		rightDrive1 = new Talon(2);
		rightDrive2 = new Talon(3);
	}
	
	public void updateDriveSpeed(double leftspeed, double rightspeed)
	{
		leftDrive1.set(-leftspeed);
		leftDrive2.set(-leftspeed);
		rightDrive1.set(rightspeed);
		rightDrive2.set(rightspeed);		
	}
}
