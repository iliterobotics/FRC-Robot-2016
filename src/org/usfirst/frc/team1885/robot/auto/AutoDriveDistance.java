package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

/**
 * Waits until the robot has traversed a certain distance. Moving forward 1 in
 * then backwards 1 equates to traveling 2 in. This is determined by the left
 * wheel's encoder values and the circumference of the wheels. Generic encoder
 * to distance math is used.
 * 
 * @author ILITE Robotics
 * @version <2/13/2016>
 */
public class AutoDriveDistance extends AutoCommand {
    private SensorInputControlSRX sensorInputControl;
    private double distance;
    private double initDisLeft;
    private double initDisRight;
    private double disLeft;
    private double disRight;
    private double leftDriveSpeed;
    private double rightDriveSpeed;

    /**
     * @param d
     *            Traverse distance in inches
     */
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
