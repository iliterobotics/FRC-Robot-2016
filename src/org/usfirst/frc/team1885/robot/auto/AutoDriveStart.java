package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Drives the Drive Train in a straight direction.
 * 
 * @author ILITE Robotics
 * @version <2/13/2016>
 */
public class AutoDriveStart extends AutoCommand {

    private double time;
    private double leftDriveOutput;
    private double rightDriveOutput;

    private static double MIN_SPEED = 0.0;

    /**
     * Sends equal power to both sides of the Drive Train for specified amount
     * of time, then stops sending power. No other actions can be done during
     * this time.
     * 
     * @param pow
     *            Power from [-1, 1], or from -100% to 100%
     * @param sec
     *            Time in seconds
     */
    public AutoDriveStart(double pow, double sec) {
        SensorInputControlSRX.getInstance().calibrateGyro();
        rightDriveOutput = leftDriveOutput = pow;
        time = sec;
    }

    /**
     * Sends equal power to both sides of the Drive Train. Does not stop sending
     * power.
     * 
     * @param pow
     *            Power from [-1, 1], or from -100% to 100%
     */
    public AutoDriveStart(double pow) {
        SensorInputControlSRX.getInstance().calibrateGyro();
        rightDriveOutput = leftDriveOutput = pow;
        time = 0;
        init();
    }

    public boolean execute() {
        Timer.delay(time);
        DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveOutput);
        DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveOutput);
        updateOutputs();
        if (time != 0) {
            Timer.delay(time);
            reset();
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
