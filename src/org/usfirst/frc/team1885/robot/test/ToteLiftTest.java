package org.usfirst.frc.team1885.robot.test;

import org.usfirst.frc.team1885.robot.auto.AutoWait;
import org.usfirst.frc.team1885.robot.modules.lift.ToteLift;

public class ToteLiftTest {
	private AutoWait waitForIt = new AutoWait(2000);
	public void test()
	{
		liftIt(12, .5);
		stopIt();
		liftIt(12, -.5);
		stopIt();
		ToteLift.getInstance().updateLift(0.42);
		ToteLift.getInstance().cycle();
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
