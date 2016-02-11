package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AutoStartDrive extends AutoCommand {

    private double time;
    private double leftDriveOutput;
    private double rightDriveOutput;

    private static double MIN_SPEED = 0.0;

    public AutoStartDrive(double sec, double pow) {
        rightDriveOutput = leftDriveOutput = -pow;

        time = sec;
        DriverStation.reportError(
                "Drive with " + pow + " power for " + sec + " seconds", false);
        init();
    }

    public AutoStartDrive(double pow) {
        SensorInputControlSRX.getInstance().calibrateGyro();
        rightDriveOutput = leftDriveOutput = -pow;
        time = 0;
        init();
    }

    public boolean execute() {
        DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveOutput);
        DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveOutput);
        updateOutputs();
        if (time != 0) {
            Timer.delay(time);
            reset();
            updateOutputs();
        }
        return true;
    }
    public void reset() {
        DrivetrainControl.getInstance().setLeftDriveSpeed(0);
        DrivetrainControl.getInstance().setRightDriveSpeed(0);
    }

    public boolean updateOutputs() {
        RobotControlWithSRX.getInstance().updateDriveSpeed(
                DrivetrainControl.getInstance().getLeftDriveSpeed(),
                DrivetrainControl.getInstance().getRightDriveSpeed());
        return true;
    }

    public boolean init() {
        reset();
        return true;
    }

}
