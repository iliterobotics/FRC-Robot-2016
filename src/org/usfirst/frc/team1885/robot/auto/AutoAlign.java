package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class rotates the robot into it's initial facing - yaw - position.
 * 'Initial facing' position being the position at which the robot was when this
 * class was created. It rotates to that position by anchoring one side of the
 * drive train and negatively powering the other.
 * 
 * @author ILITE Robotics
 * @version 2/13/2016
 */
public class AutoAlign extends AutoCommand {

    private final double SPEED = 0.2;
    private final double ALIGNMENT_ERROR = 1;
    private SensorInputControlSRX sensorInputControl;
    private double rightDrivePower;
    private double leftDrivePower;
    private double initial_yaw; // Yaw before we start aligning

    @Override
    public boolean init() {
        rightDrivePower = leftDrivePower = 0;
        sensorInputControl = SensorInputControlSRX.getInstance();
        initial_yaw = sensorInputControl.getYaw();
        return true;
    }

    @Override
    public boolean execute() {
        double yaw = sensorInputControl.getYaw();

        rightDrivePower = leftDrivePower = SPEED;

        if (yaw > ALIGNMENT_ERROR) {
            leftDrivePower = -leftDrivePower;
            rightDrivePower = 0;
        } else if (yaw < -ALIGNMENT_ERROR) {
            rightDrivePower = -rightDrivePower;
            leftDrivePower = 0;
        } else {
            DriverStation.reportError("Alligned.", false);
            leftDrivePower = 0;
            rightDrivePower = 0;
            return true;
        }
        DrivetrainControl.getInstance().setLeftDriveSpeed(leftDrivePower);
        DrivetrainControl.getInstance().setRightDriveSpeed(rightDrivePower);

        return false;
    }

    @Override
    public boolean updateOutputs() {
        RobotControlWithSRX.getInstance().updateDriveSpeed(leftDrivePower,
                rightDrivePower);
        return false;
    }

    @Override
    public void reset() {
        DrivetrainControl.getInstance().setLeftDriveSpeed(0);
        DrivetrainControl.getInstance().setRightDriveSpeed(0);
    }

}
