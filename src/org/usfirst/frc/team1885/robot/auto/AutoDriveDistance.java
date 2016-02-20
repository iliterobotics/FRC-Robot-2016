package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
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
    private double differenceLeft, differenceRight;

    private PID leftPID, rightPID;
    private double P, I, D;

    private final double MIN_SPEED;
    private final double ERROR;

    public AutoDriveDistance() {
        MIN_SPEED = 0.2;
        ERROR = 4;
        P = .8;
        I = 0.005;
        D = 5;
        leftPID = new PID(P, I, D);
        rightPID = new PID(P, I, D);
        differenceLeft = differenceRight = 0;
    }

    /**
     * @param d
     *            Distance to travel in inches
     * @param b
     *            If it should stop at the end of the distance
     */
    public AutoDriveDistance(double d, boolean b) {
        this();
        leftPID.setScalingValue(d);
        rightPID.setScalingValue(d);
        sensorInputControl = SensorInputControlSRX.getInstance();
        distance = d;
        doesStop = b;
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
                .getEncoderDistance(SensorType.LEFT_ENCODER) - initDisLeft;
        disRight = sensorInputControl
                .getEncoderDistance(SensorType.RIGHT_ENCODER) - initDisRight;

        // DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveSpeed);
        // DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveSpeed);

        differenceLeft = disLeft - distance;
        differenceRight = disRight - distance;

        leftDriveSpeed = leftPID.getPID(distance, disLeft);
        rightDriveSpeed = rightPID.getPID(distance, disRight);

        isLeftFinished = Math.abs(differenceLeft) < ERROR;
        isRightFinished = Math.abs(differenceRight) < ERROR;

        DriverStation.reportError("\n\nDistance Left:: " + disLeft
                + "\ndistance Right:: " + disRight + "\nDifference Left:: "
                + differenceLeft + "\nDifference Right:: " + differenceRight,
                false);

        if (leftDriveSpeed > 0) {
            leftDriveSpeed = leftDriveSpeed < MIN_SPEED ? MIN_SPEED
                    : leftDriveSpeed;
        } else if (leftDriveSpeed < 0) {
            leftDriveSpeed = leftDriveSpeed > -MIN_SPEED ? -MIN_SPEED
                    : leftDriveSpeed;
        }

        if (rightDriveSpeed > 0) {
            rightDriveSpeed = rightDriveSpeed < MIN_SPEED ? MIN_SPEED
                    : rightDriveSpeed;
        } else if (rightDriveSpeed < 0) {
            rightDriveSpeed = rightDriveSpeed > -MIN_SPEED ? -MIN_SPEED
                    : rightDriveSpeed;
        }

        DriverStation.reportError("\nRight Drive Speed:: " + rightDriveSpeed + "\nLeft Drive Speed:: " + leftDriveSpeed, false);
        
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
        initDisLeft = sensorInputControl
                .getEncoderDistance(SensorType.LEFT_ENCODER);
        initDisRight = sensorInputControl
                .getEncoderDistance(SensorType.RIGHT_ENCODER);

        if (leftInputPower == 0 && rightInputPower == 0) {
            leftInputPower = DrivetrainControl.getInstance()
                    .getLeftDriveSpeed();
            rightInputPower = DrivetrainControl.getInstance()
                    .getRightDriveSpeed();
        }
        leftDriveSpeed = rightInputPower;
        rightDriveSpeed = leftInputPower;
        return true;
    }

    @Override
    public void reset() {
        // No values to reset

    }

}
