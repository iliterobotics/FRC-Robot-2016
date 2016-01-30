package org.usfirst.frc.team1885.robot.test;

import org.usfirst.frc.team1885.graveyard.DriverInputControl;
import org.usfirst.frc.team1885.graveyard.ToteLift;
import org.usfirst.frc.team1885.robot.auto.AutoWait;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;

public class ToteLiftTest {
	private AutoWait waitForIt = new AutoWait(2000);
	public void test()
	{
		if (DriverInputControl.getInstance().getButton(
				RobotButtonType.TOTE_LIFT_TEST))
		{
		liftIt(12, .5);
		stopIt();
		liftIt(12, -.5);
		stopIt();
		ToteLift.getInstance().updateLift(0.42);
		}
	}
	public void stopIt()
	{
		while(!waitForIt.execute())
		{
			waitForIt.execute();
		}
	}
	public void liftIt(double dist, double error)
	{
		ToteLift.getInstance().incrementLift(dist, error);
	}
}
