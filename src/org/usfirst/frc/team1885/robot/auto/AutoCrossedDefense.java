package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Waits until the robot has crossed over a defense. This is determined by if
 * the robot has been level with the ground for a certain amount of time.
 * 
 * @author ILITE Robotics
 * @version <2/13/2016>
 */
public class AutoCrossedDefense extends AutoCommand {

    private final double ERROR; // Margin of error for Pitch and Roll
    private final double FLAT_PITCH;// Initial Pitch of robot on creation of
                                    // class
    private final double FLAT_ROLL;// Initial Roll of robot on creation of class
    private final double WAIT_TIME;// Time (in seconds) the robot must be
                                   // 'flat' to be considered on flat ground

    private SensorInputControlSRX sensorInputControl;
    private long startTime;
    private double leftDriveSpeed;
    private double rightDriveSpeed;

    public AutoCrossedDefense() {
        sensorInputControl = SensorInputControlSRX.getInstance();
        ERROR = 2;
        WAIT_TIME = 0.15;
        FLAT_ROLL = sensorInputControl.getInitRoll();
        FLAT_PITCH = sensorInputControl.getInitPitch();
    }

    @Override
    public boolean init() {
        startTime = System.currentTimeMillis();
        leftDriveSpeed = DrivetrainControl.getInstance().getLeftDriveSpeed();
        rightDriveSpeed = DrivetrainControl.getInstance().getRightDriveSpeed();
        return true;
    }

    @Override
    public boolean execute() {
        double currentRoll = sensorInputControl.getNavX().getRoll();
        double currentPitch = sensorInputControl.getNavX().getPitch();

        DriverStation.reportError("\nInitial Roll: " + FLAT_ROLL
                + ":::Initial Pitch: " + FLAT_PITCH + "\nCurrent Roll: "
                + currentRoll + ":::Current Pitch: " + currentPitch, false);

        boolean isAlignedRoll = currentRoll <= FLAT_ROLL + ERROR
                && currentRoll >= FLAT_ROLL - ERROR;

        boolean isAlignedPitch = currentPitch <= FLAT_PITCH + ERROR
                && currentPitch >= FLAT_PITCH - ERROR;

        if (isAlignedPitch && isAlignedRoll) {
            if (System.currentTimeMillis() - startTime > WAIT_TIME * 1000) {
                // WAIT_TIME converted to millis
                leftDriveSpeed = rightDriveSpeed = 0;
                DrivetrainControl.getInstance()
                        .setLeftDriveSpeed(leftDriveSpeed);
                DrivetrainControl.getInstance()
                        .setRightDriveSpeed(rightDriveSpeed);

                DriverStation.reportError(
                        "\nCrossed the defense, stopping robot now."
                                + System.currentTimeMillis(),
                        false);
                return true;
            }
        } else {
            startTime = System.currentTimeMillis();
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
    public void reset() {
        // No values to reset
    }

}
