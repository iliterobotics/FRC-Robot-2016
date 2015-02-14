package org.usfirst.frc.team1885.robot.test;

import org.usfirst.frc.team1885.robot.auto.AutoWait;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.modules.lift.ToteLift;

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
		ToteLift.getInstance().cycle();
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
