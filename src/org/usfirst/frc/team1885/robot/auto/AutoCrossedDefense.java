package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoCrossedDefense extends AutoCommand {
    // autonomous checkpoint used to check anything after we cross a defense and
    // when to stop

    private final double FLAT_STANDARD; // Error allowed to be flat
    private final double FLAT_PITCH;
    private final double FLAT_ROLL;
    private final double WAIT_TIME;

    private SensorInputControlSRX sensorInputControl;
    private long startTime;
    private double leftDriveSpeed;
    private double rightDriveSpeed;

    public AutoCrossedDefense() {
        sensorInputControl = SensorInputControlSRX.getInstance();
        FLAT_STANDARD = .5;
        WAIT_TIME = 0.5;
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
        DriverStation.reportError(
                "\nRoll: " + sensorInputControl.getNavX().getRoll()
                        + " ::: Pitch: "
                        + sensorInputControl.getNavX().getPitch()
                        + "\t\t Target Roll: " + FLAT_ROLL + FLAT_STANDARD
                        + " ::: Target Pitch: " + FLAT_PITCH + FLAT_STANDARD,
                false);

        boolean isAlignedRoll = sensorInputControl.getNavX()
                .getRoll() <= FLAT_ROLL + FLAT_STANDARD
                && sensorInputControl.getNavX().getRoll() >= FLAT_ROLL
                        - FLAT_STANDARD;

        boolean isAlignedPitch = sensorInputControl.getNavX()
                .getPitch() <= FLAT_PITCH + FLAT_STANDARD
                && sensorInputControl.getNavX().getPitch() >= FLAT_PITCH
                        - FLAT_STANDARD;

        if (isAlignedPitch && isAlignedRoll) {
            if (System.currentTimeMillis() - startTime > WAIT_TIME) {
                leftDriveSpeed = rightDriveSpeed = 0;
                DriverStation.reportError("\n\t\t\t\tAlligned...", false);
                // DrivetrainControl.getInstance()
                // .setLeftDriveSpeed(leftDriveSpeed);
                // DrivetrainControl.getInstance()
                // .setRightDriveSpeed(rightDriveSpeed);
            }
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        // RobotControlWithSRX.getInstance().updateDriveSpeed(leftDriveSpeed,
        // rightDriveSpeed);
        return false;
    }

    @Override
    public void reset() {
        // No values to reset
    }

}
