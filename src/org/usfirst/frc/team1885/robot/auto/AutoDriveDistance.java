package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

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
        DriverStation.reportError("Initial distance: " + initDisLeft, false);
    }

    @Override
    public boolean execute() {
        disLeft = Math.abs(
                sensorInputControl.getEncoderDistance(SensorType.LEFT_ENCODER));
        disRight = Math.abs(
                sensorInputControl.getEncoderDistance(SensorType.LEFT_ENCODER));
        DriverStation.reportError("\nDistance of left side traveled: " + disLeft
                + "\nDistance of right side traveled: " + disRight + "\n",
                false);
        if (disLeft >= distance) {
            DriverStation.reportError("Finished traveling distance!", false);
            leftDriveSpeed = 0;
            rightDriveSpeed = 0;
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
        // leftDriveSpeed = DrivetrainControl.getInstance().getLeftDriveSpeed();
        // rightDriveSpeed =
        // DrivetrainControl.getInstance().getRightDriveSpeed();
        leftDriveSpeed = -.7;
        rightDriveSpeed = -.7;
        return true;
    }

    @Override
    public void reset() {
        // No values to reset

    }

}
