package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

/**
 * This class crosses the Rampart defense by shimmy-ing up the ramps. The ramps
 * must be angled toward the ground beforehand.
 * 
 * @author ILITE Robotics
 * @version <2/13/2016>
 */
public class AutoRamparts extends AutoCommand {
    // Made by Aaron. Fixed by Noah. Pushed by Teddy.
    private final double YAW_ZONE = 5.0; // Margin of error to be within in
                                         // order to be considered straight.
    private SensorInputControlSRX sensorControl = SensorInputControlSRX
            .getInstance();
    private RobotControlWithSRX robotControl = RobotControlWithSRX
            .getInstance();
    private DrivetrainControl driveTrainControl = DrivetrainControl
            .getInstance();
    private double leftDriveSpeed;
    private double rightDriveSpeed;

    @Override
    public boolean init() {
        reset();
        leftDriveSpeed = driveTrainControl.getLeftDriveSpeed();
        rightDriveSpeed = driveTrainControl.getRightDriveSpeed();
        return true;
    }

    @Override
    public boolean execute() {
        double yaw = sensorControl.getYaw();
        if ((new AutoCrossedDefense()).execute()) {
            return true;
        }
        if (this.inZone(yaw)) {
            leftDriveSpeed = rightDriveSpeed = AutonomousRoutine.RAMPART_SPEED_MAX;
        } else if (yaw > YAW_ZONE) {
            if (driveTrainControl
                    .getLeftDriveSpeed() == AutonomousRoutine.RAMPART_SPEED_MAX) {
                leftDriveSpeed = AutonomousRoutine.RAMPART_SPEED_MIN;
            }
            rightDriveSpeed = AutonomousRoutine.RAMPART_SPEED_MAX;
        } else if (yaw < -YAW_ZONE) {
            if (driveTrainControl
                    .getRightDriveSpeed() == AutonomousRoutine.RAMPART_SPEED_MAX) {
                rightDriveSpeed = AutonomousRoutine.RAMPART_SPEED_MIN;
            }
            leftDriveSpeed = AutonomousRoutine.RAMPART_SPEED_MAX;
        }
        driveTrainControl.setLeftDriveSpeed(leftDriveSpeed);
        driveTrainControl.setRightDriveSpeed(rightDriveSpeed);
        return false;
    }

    @Override
    public boolean updateOutputs() {
        robotControl.updateDriveSpeed(leftDriveSpeed, rightDriveSpeed);
        return true;
    }

    @Override
    public void reset() {
        robotControl.updateDriveSpeed(0, 0);
    }

    /**
     * Determines whether or not the Drive Train is 'straight enough' to cross
     * over the Ramparts.
     * 
     * @param input
     *            Current yaw value.
     * @return True when input value is within (-YAW_ZONE, YAW_ZONE). False
     *         otherwise.
     */
    public boolean inZone(double input) {
        if (input < YAW_ZONE && input > -YAW_ZONE) {
            return true;
        }
        return false;
    }
}
