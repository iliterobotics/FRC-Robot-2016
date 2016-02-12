package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AutoAlign extends AutoCommand {

    private final double P = 0.6;
    private final double I = 0.02;
    private final double D = 0;
 
    private PID pid;
    private SensorInputControlSRX sensorInputControl;
    private double rightDriveSpeed;
    private double leftDriveSpeed;
    private double rightDistanceTraveled;
    private double initial_yaw;

    private static double MIN_SPEED = 0.15;

    @Override
    public boolean init() {
        rightDriveSpeed = leftDriveSpeed = 0;
        sensorInputControl = SensorInputControlSRX.getInstance();
        initial_yaw = sensorInputControl.getYaw();
        return true;
    }

    @Override
    public boolean execute() {
        double yaw = sensorInputControl.getYaw();

        leftDriveSpeed = pid.getPID(0, initial_yaw - yaw);

        if (leftDriveSpeed > 0) {
            leftDriveSpeed = (leftDriveSpeed < AutoAlign.MIN_SPEED
                    ? AutoAlign.MIN_SPEED : leftDriveSpeed);
        } else if (leftDriveSpeed < 0) {
            leftDriveSpeed = (leftDriveSpeed > -AutoAlign.MIN_SPEED
                    ? -AutoAlign.MIN_SPEED : leftDriveSpeed);
        }

        rightDriveSpeed = pid.getPID(0, initial_yaw - yaw);

        if (rightDriveSpeed > 0) {
            rightDriveSpeed = (rightDriveSpeed < AutoAlign.MIN_SPEED
                    ? AutoAlign.MIN_SPEED : rightDriveSpeed);
        } else if (rightDriveSpeed < 0) {
            rightDriveSpeed = (rightDriveSpeed > -AutoAlign.MIN_SPEED
                    ? -AutoAlign.MIN_SPEED : rightDriveSpeed);
        }

        if (yaw > 0) {
            leftDriveSpeed = -leftDriveSpeed;
            rightDriveSpeed = 0;
        } else if (yaw < 0) {
            rightDriveSpeed = -rightDriveSpeed;
            leftDriveSpeed = 0;
        }

        // System.out.println("AutoDriveFwd::[left speed, right speed] " +
        // leftDriveOutput + ", " + rightDriveOutput);

        DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveSpeed);
        DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveSpeed);
        return false;
    }

    @Override
    public boolean updateOutputs() {
        RobotControlWithSRX.getInstance().updateDriveSpeed(leftDriveSpeed,
                rightDriveSpeed);
        return false;
    }

    @Override
    public void reset() {
        pid.reset();
        DrivetrainControl.getInstance().setLeftDriveSpeed(0);
        DrivetrainControl.getInstance().setRightDriveSpeed(0);
    }

}
