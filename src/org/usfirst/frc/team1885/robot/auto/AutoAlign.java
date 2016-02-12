package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoAlign extends AutoCommand {

    private final double P = 0.6;
    private final double I = 0.02;
    private final double D = 0;
    private final double ALIGNMENT_ERROR = 1;

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
        pid = new PID(.35, .002, 0); //Not final values
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

        if (yaw > ALIGNMENT_ERROR) {
            leftDriveSpeed = -leftDriveSpeed;
            rightDriveSpeed = 0;
        } else if (yaw < -ALIGNMENT_ERROR) {
            rightDriveSpeed = -rightDriveSpeed;
            leftDriveSpeed = 0;
        } else {
            reset();
            return true;
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
        DrivetrainControl.getInstance().setLeftDriveSpeed(0);
        DrivetrainControl.getInstance().setRightDriveSpeed(0);
        pid.reset();
    }

}
