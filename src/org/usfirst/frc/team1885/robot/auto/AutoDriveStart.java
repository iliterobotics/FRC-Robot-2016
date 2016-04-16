package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
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
    private DrivetrainControl drivetrainControl;

    /**
     * Sends equal power to both sides of the Drive Train for specified amount
     * of time, then stops sending power. No other actions can be done during
     * this time.
     * 
     * @param sec
     *            Time in seconds
     * @param speed
     *            Power from [-1, 1], or from -100% to 100% of max power
     */
    public AutoDriveStart(double sec, double speed) {
        this(speed);
        time = sec;
    }

    /**
     * Sends equal power to both sides of the Drive Train. Does not stop sending
     * power.
     * 
     * @param speed
     *            Power from [-1, 1], or from -100% to 100% of max power
     */
    public AutoDriveStart(double speed) {
        drivetrainControl = DrivetrainControl.getInstance();
        rightDriveOutput = leftDriveOutput = speed;
        time = 0;
        init();
    }

    public boolean execute() {
        drivetrainControl.setControlMode(TalonControlMode.Speed);
        reset();
        drivetrainControl.setLeftDriveSpeed(leftDriveOutput);
        drivetrainControl.setRightDriveSpeed(rightDriveOutput);
        if (time != 0) {
            Timer.delay(time);
            reset();
        }
        return true;
    }
    public void reset() {
        drivetrainControl.setLeftDriveSpeed(0);
        drivetrainControl.setRightDriveSpeed(0);
    }

    public boolean updateOutputs() {
        drivetrainControl.updateOutputs();
        return true;
    }

    public boolean init() {
        return true;
    }

}
