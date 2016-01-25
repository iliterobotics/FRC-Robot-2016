package org.usfirst.
frc.team1885.robot.test;

import org.usfirst.frc.team1885.graveyard.AutoDriveForward;
import org.usfirst.frc.team1885.graveyard.AutoTurn;
import org.usfirst.frc.team1885.graveyard.AutoWait;
import org.usfirst.frc.team1885.graveyard.DriverInputControl;
import org.usfirst.frc.team1885.robot.common.type.GearState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

public class DriveTrainTest {
	private int inches = 12; 
	private double error = 0.5;
	private AutoDriveForward driveFward; 
	private AutoTurn turner;
	private DrivetrainControl gear;
	private AutoWait waitForIt;
	//private boolean toggle;
	public DriveTrainTest()
	{
		driveFward = new AutoDriveForward(inches, error, 2);
		turner = new AutoTurn(45, 1);
		waitForIt = new AutoWait(5000);
	}
	
	public void testDrive()
	{
		if (DriverInputControl.getInstance().getButton(
				RobotButtonType.DRIVE_TRAIN_TEST))
				{
		driveOn();
		driveFward = new AutoDriveForward(-inches, error, 2);
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
