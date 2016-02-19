package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;

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

    private double distance; // Inputed distance to traverse
    private double initDisLeft; // Initial distance of left drive train side
    private double initDisRight; // Initial distance of right drive train side
    private double disLeft; // Current distance of left drive train side
    private double disRight; // Current distance of right drive train side
    private double leftDriveSpeed; // Power given to left drive train side
    private double rightDriveSpeed; // Power given to right drive train side
    private double leftInputPower; // Inputed power of left side to travel at
    private double rightInputPower; // Inputed power of right side to travel at

    private boolean doesStop; // If the drive train should stop after traversing
    private boolean isLeftFinished; // If the left drive train side is finished
                                    // traversing
    private boolean isRightFinished; // If the right drive train side is
                                     // finished traversing
    private AutoDriveForward driveForward;

    /**
     * @param d
     *            Distance to travel in inches
     * @param b
     *            If it should stop at the end of the distance
     */
    public AutoDriveDistance(double d, boolean b) {
        sensorInputControl = SensorInputControlSRX.getInstance();
        distance = d;
        doesStop = b;
        driveForward = new AutoDriveForward(SensorType.LEFT_ENCODER,
                SensorType.RIGHT_ENCODER);
    }

    /**
     * @param d
     *            Distance to travel in inches
     * @param b
     *            If it should stop at the end of the distance
     * @param lP
     *            Power of left side of drive train
     * @param rP
     *            Power of right side of drive train
     */
    public AutoDriveDistance(double d, boolean b, double lP, double rP) {
        this(d, b);
        leftInputPower = lP;
        rightInputPower = rP;
    }

    @Override
    public boolean execute() {
        disLeft = sensorInputControl
                .getEncoderDistance(SensorType.LEFT_ENCODER);
        disRight = sensorInputControl
                .getEncoderDistance(SensorType.RIGHT_ENCODER);

//        DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveSpeed);
//        DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveSpeed);

        isLeftFinished = Math.abs(disLeft - initDisLeft) >= distance;
        isRightFinished = Math.abs(disRight - initDisRight) >= distance;

        // DriverStation.reportError(
        // "\nDisRight: " + disRight + ", initDisRight: " + initDisRight,
        // false);
        // DriverStation.reportError(
        // "\ndisLeft: " + disLeft + ", initDisLeft: " + initDisLeft,
        // false);

        if (!doesStop && isRightFinished && isLeftFinished) {
            DriverStation.reportError("\nFinished traveling distance!"
                    + System.currentTimeMillis(), false);
            return true;
        } else if (isRightFinished && isLeftFinished) {
            DriverStation.reportError(
                    "\nFinished traveling distance! Stopping.", false);
            leftDriveSpeed = rightDriveSpeed = 0;
            DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveSpeed);
            DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveSpeed);
            return true;
        } else if (isLeftFinished) {
            leftDriveSpeed = 0;
        } else if (isRightFinished) {
            rightDriveSpeed = 0;
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        driveForward.driveForward(leftDriveSpeed, rightDriveSpeed);
        return false;
    }

    @Override
    public boolean init() {
        if (leftInputPower == 0 && rightInputPower == 0) {
            leftInputPower = DrivetrainControl.getInstance()
                    .getLeftDriveSpeed();
            rightInputPower = DrivetrainControl.getInstance()
                    .getRightDriveSpeed();
        }
        initDisLeft = sensorInputControl
                .getEncoderDistance(SensorType.LEFT_ENCODER);
        initDisRight = sensorInputControl
                .getEncoderDistance(SensorType.RIGHT_ENCODER);
        leftDriveSpeed = rightInputPower;
        rightDriveSpeed = leftInputPower;
        return true;
    }

    @Override
    public void reset() {
        // No values to reset

    }

}
