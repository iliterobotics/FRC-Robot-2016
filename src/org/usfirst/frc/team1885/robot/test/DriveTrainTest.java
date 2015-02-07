package org.usfirst.
frc.team1885.robot.test;

import org.usfirst.frc.team1885.robot.auto.AutoDriveForward;
import org.usfirst.frc.team1885.robot.auto.AutoTurn;
import org.usfirst.frc.team1885.robot.auto.AutoWait;
import org.usfirst.frc.team1885.robot.common.type.GearState;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

public class DriveTrainTest {
	private int inches = 12; 
	private double error = 0.5;
	private AutoDriveForward driveFward; 
	private AutoTurn turner;
	private DrivetrainControl gear;
	private AutoWait waitForIt;
	public DriveTrainTest()
	{
		driveFward = new AutoDriveForward(inches, error);
		turner = new AutoTurn(45, 1);
		waitForIt = new AutoWait(5000);
	}
	public void testDrive()
	{
		driveOn();
		driveFward = new AutoDriveForward(-inches, error);
		driveOn();
		turnIt();
		turner = new AutoTurn(-45, 5);
		turnIt();
		turner = new AutoTurn(90, 5);
		turnIt();
		turner = new AutoTurn(-90, 5);
		turnIt();
		turner = new AutoTurn(180, 5);
		turnIt();
		turner = new AutoTurn(-180, 5);
		gear.setGearState(GearState.HIGH_GEAR);
		stopIt();
		gear.setGearState(GearState.LOW_GEAR);
		stopIt();
		gear.setGearState(GearState.HIGH_GEAR);
	}
	public void stopIt()
	{
		while(!waitForIt.execute())
		{
			waitForIt.execute();
		}
	}
	public void driveOn()
	{
		while (!driveFward.execute())
		{
			driveFward.execute();
		}
	}
	public void turnIt()
	{
		while (!turner.execute())
		{
			turner.execute();
		}
	}
}
