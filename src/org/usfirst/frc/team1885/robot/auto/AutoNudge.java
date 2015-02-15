package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class AutoNudge extends AutoCommand {

	private final double MAX_SPEED = .25;

	public boolean execute() {
		while(SensorInputControl.getInstance().isActive(SensorType.TOUCH_SENSOR_TOTE_LEFT)) {
			DrivetrainControl.getInstance().setLeftDriveSpeed(-MAX_SPEED);
		}
		
		if(SensorInputControl.getInstance().isActive(SensorType.TOUCH_SENSOR_TOTE_RIGHT)) {
			DrivetrainControl.getInstance().setRightDriveSpeed(-MAX_SPEED);
		}
		return true;
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
