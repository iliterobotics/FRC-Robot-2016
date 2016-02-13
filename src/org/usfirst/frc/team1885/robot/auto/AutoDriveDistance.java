package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AutoDriveDistance extends AutoCommand {
    private SensorInputControlSRX sensorInputControl;
    private double distance;
    private double initDisLeft;
    private double initDisRight;
    private double disLeft;
    private double disRight;
    private double leftDriveSpeed;
    private double rightDriveSpeed;

    // Distance should be in inches
    public AutoDriveDistance(double d) {
        sensorInputControl = SensorInputControlSRX.getInstance();
        distance = d;
        initDisLeft = Math.abs(
                sensorInputControl.getEncoderDistance(SensorType.LEFT_ENCODER));
    }

    @Override
    public boolean execute() {
        disLeft = Math.abs(
                sensorInputControl.getEncoderDistance(SensorType.LEFT_ENCODER));

        if (disLeft - initDisLeft >= distance) {
            return true;
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        RobotControlWithSRX.getInstance().updateDriveSpeed(leftDriveSpeed,
                rightDriveSpeed);
        return false;
    }

    @Override
    public boolean init() {
        leftDriveSpeed = DrivetrainControl.getInstance().getLeftDriveSpeed();
        rightDriveSpeed = DrivetrainControl.getInstance().getRightDriveSpeed();
        return false;
    }

    @Override
    public void reset() {
        // No values to reset

    }

}
