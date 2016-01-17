package org.usfirst.frc.team1885.robot.test;

import org.usfirst.frc.team1885.graveyard.RobotControl;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class RobotOutputTest 
{
	RobotControl robotControl = RobotControl.getInstance();
	public RobotOutputTest()
	{
		robotControl.addTalonOutput(RobotMotorType.LEFT_DRIVE, 0);
		robotControl.addTalonOutput(RobotMotorType.LEFT_DRIVE, 1);
		robotControl.addTalonOutput(RobotMotorType.RIGHT_DRIVE, 2);
		robotControl.addTalonOutput(RobotMotorType.RIGHT_DRIVE, 3);	
	}
	public void outputTest()
	{
	}
	
}
