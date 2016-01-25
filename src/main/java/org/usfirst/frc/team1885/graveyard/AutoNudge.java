package org.usfirst.frc.team1885.graveyard;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

public class AutoNudge extends AutoCommand {

	private final double MAX_SPEED = .25;

	public boolean execute() {
		
		boolean bDriveLeft = SensorInputControl.getInstance().isActive(SensorType.TOUCH_SENSOR_TOTE_LEFT);
		boolean bDriveRight = SensorInputControl.getInstance().isActive(SensorType.TOUCH_SENSOR_TOTE_RIGHT);
				
		if(bDriveLeft) {
			DrivetrainControl.getInstance().setLeftDriveSpeed(-MAX_SPEED);
		}
		else
		{
			DrivetrainControl.getInstance().setLeftDriveSpeed(0);
		}
		
		if(bDriveRight) {
			DrivetrainControl.getInstance().setRightDriveSpeed(-MAX_SPEED);
		}
		else
		{
			DrivetrainControl.getInstance().setRightDriveSpeed(0);
		}
		
		return !bDriveLeft && !bDriveRight;
	}

	public boolean updateOutputs() {
		RobotControl.getInstance().updateDriveSpeed(DrivetrainControl.getInstance().getLeftDriveSpeed(), DrivetrainControl.getInstance().getRightDriveSpeed());
		return true;
	}
	public boolean init() {
		reset();
		return true;
	}
	public void reset() {
		RobotControl.getInstance().updateDriveSpeed(0, 0);
	}

}
