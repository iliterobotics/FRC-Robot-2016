package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class crosses the Rampart defense by shimmy-ing up the ramps. The ramps
 * must be angled toward the ground beforehand.
 * 
 * @author ILITE Robotics
 * @version <2/13/2016>
 */
public class AutoRamparts extends AutoCommand {
    private final double YAW_ZONE = 5.0; // Margin of error to be within in
                                         // order to be considered straight.
    private final double RAMPART_SPEED_MAX = 0.6; // percentage of max speed
    private final double RAMPART_SPEED_MIN = 0.5; // percentage of max speed
    private SensorInputControlSRX sensorControl = SensorInputControlSRX
            .getInstance();
    private RobotControlWithSRX robotControl = RobotControlWithSRX
            .getInstance();
    private DrivetrainControl driveTrainControl = DrivetrainControl
            .getInstance();
    private AutoCrossedDefense crossedDefense;
    private double leftDriveSpeed;
    private double rightDriveSpeed;

    @Override
    public boolean init() {
        DrivetrainControl.getInstance().setControlMode(TalonControlMode.Speed);
        reset();
        leftDriveSpeed = driveTrainControl.getLeftDriveSpeed();
        rightDriveSpeed = driveTrainControl.getRightDriveSpeed();
        crossedDefense = new AutoCrossedDefense();
        return true;
    }

    @Override
    public boolean execute() {
        double yaw = sensorControl.getYaw();
//        DriverStation.reportError("\nRight side: " + leftDriveSpeed
//                + " --- Left side: " + rightDriveSpeed, false);
        if (crossedDefense.execute()) {
            return true;
        }
        if (this.inZone(yaw)) {
            leftDriveSpeed = rightDriveSpeed = RAMPART_SPEED_MAX;
        } else if (yaw > YAW_ZONE) {
            if (driveTrainControl.getLeftDriveSpeed() == RAMPART_SPEED_MAX) {
                leftDriveSpeed = RAMPART_SPEED_MIN;
            }
            rightDriveSpeed = RAMPART_SPEED_MAX;
        } else if (yaw < -YAW_ZONE) {
            if (driveTrainControl.getRightDriveSpeed() == RAMPART_SPEED_MAX) {
                rightDriveSpeed = RAMPART_SPEED_MIN;
            }
            leftDriveSpeed = RAMPART_SPEED_MAX;
        }
        driveTrainControl.setLeftDriveSpeed(leftDriveSpeed);
        driveTrainControl.setRightDriveSpeed(rightDriveSpeed);
        return false;
    }

    @Override
    public boolean updateOutputs() {
        DrivetrainControl.getInstance().updateOutputs();
        return true;
    }

    @Override
    public void reset() {
        robotControl.updateDriveSpeed(driveTrainControl.getLeftDriveSpeed(),
                driveTrainControl.getRightDriveSpeed());
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
