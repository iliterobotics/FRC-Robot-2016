package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
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

    private final double P = 0.6;
    private final double I = 0.02;
    private final double D = 0;
    private final double ALIGNMENT_ERROR = 1;

    private PID pid;
    private SensorInputControlSRX sensorInputControl;
    private double rightDrivePower;
    private double leftDrivePower;
    private double initial_yaw; // Yaw obtained at creation of class

    private static double MIN_SPEED = 0.15;

    @Override
    public boolean init() {
        rightDrivePower = leftDrivePower = 0;
        sensorInputControl = SensorInputControlSRX.getInstance();
        initial_yaw = sensorInputControl.getYaw();
        pid = new PID(.35, .002, 0); // Test for appropriate final values
        return true;
    }

    @Override
    public boolean execute() {
        double yaw = sensorInputControl.getYaw();

        leftDrivePower = pid.getPID(0, initial_yaw - yaw);

        if (leftDrivePower > 0) {
            leftDrivePower = (leftDrivePower < AutoAlign.MIN_SPEED
                    ? AutoAlign.MIN_SPEED : leftDrivePower);
        } else if (leftDrivePower < 0) {
            leftDrivePower = (leftDrivePower > -AutoAlign.MIN_SPEED
                    ? -AutoAlign.MIN_SPEED : leftDrivePower);
        }

        rightDrivePower = pid.getPID(0, initial_yaw - yaw);

        if (rightDrivePower > 0) {
            rightDrivePower = (rightDrivePower < AutoAlign.MIN_SPEED
                    ? AutoAlign.MIN_SPEED : rightDrivePower);
        } else if (rightDrivePower < 0) {
            rightDrivePower = (rightDrivePower > -AutoAlign.MIN_SPEED
                    ? -AutoAlign.MIN_SPEED : rightDrivePower);
        }

        if (yaw > ALIGNMENT_ERROR) {
            leftDrivePower = -leftDrivePower;
            rightDrivePower = 0;
        } else if (yaw < -ALIGNMENT_ERROR) {
            rightDrivePower = -rightDrivePower;
            leftDrivePower = 0;
        } else {
            reset();
            return true;
        }

        // System.out.println("AutoDriveFwd::[left speed, right speed] " +
        // leftDriveOutput + ", " + rightDriveOutput);

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
        pid.reset();
    }

}
