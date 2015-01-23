package org.usfirst.frc.team1885.robot.manipulator;

import edu.wpi.first.wpilibj.Solenoid;

public class ClawControl 
{
	private Solenoid claw;
	public ClawControl()
	{
		claw = new Solenoid(0);
	}
	public void grab()
	{
		claw.set(true);
	}
	public void release()
	{
		claw.set(false);
	}
	public boolean clawState()
	{
		return claw.get();
	}
}
