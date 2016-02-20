package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import com.sun.xml.internal.ws.api.pipe.Tube;

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

    private final double SPEED = 0.2;
    private final double ALIGNMENT_ERROR = .5;
    private final double TURN_SPEED = .46; // should be positive
    private double degreeToTurn;
    private PID pid;
    private SensorInputControlSRX sensorInputControl;
    private double rightDrivePower;
    private double leftDrivePower;

    public AutoAlign() {
        degreeToTurn = 0;
    }

    public AutoAlign(double degree) {
        degreeToTurn = degree;
    }

    @Override
    public boolean init() {
        rightDrivePower = leftDrivePower = 0;
        sensorInputControl = SensorInputControlSRX.getInstance();
        return true;
    }

    @Override
    public boolean execute() {
        double yaw = sensorInputControl.getYaw();

        DriverStation.reportError(
                "\n Degree to turn : " + degreeToTurn + " --- Current yaw: "
                        + sensorInputControl.getInstance().getYaw(),
                false);

        rightDrivePower = leftDrivePower = SPEED;

        // leftDrivePower = pid.getPID(0, -yaw);

        // if (leftDrivePower > 0) {
        // leftDrivePower = (leftDrivePower < AutoAlign.MIN_SPEED
        // ? AutoAlign.MIN_SPEED : leftDrivePower);
        // } else if (leftDrivePower < 0) {
        // leftDrivePower = (leftDrivePower > -AutoAlign.MIN_SPEED
        // ? -AutoAlign.MIN_SPEED : leftDrivePower);
        // }

        // rightDrivePower = pid.getPID(0, -yaw);

        // if (rightDrivePower > 0) {
        // rightDrivePower = (rightDrivePower < AutoAlign.MIN_SPEED
        // ? AutoAlign.MIN_SPEED : rightDrivePower);
        // } else if (rightDrivePower < 0) {
        // rightDrivePower = (rightDrivePower > -AutoAlign.MIN_SPEED
        // ? -AutoAlign.MIN_SPEED : rightDrivePower);
        // }

        if (yaw > degreeToTurn + ALIGNMENT_ERROR) {
            if (yaw < degreeToTurn + 6 + ALIGNMENT_ERROR) {
                leftDrivePower = TURN_SPEED / 2;
                rightDrivePower = -TURN_SPEED / 2;
            } else {
                leftDrivePower = TURN_SPEED;
                rightDrivePower = -TURN_SPEED;
            }
        } else if (yaw < degreeToTurn - ALIGNMENT_ERROR) {
            if (yaw > degreeToTurn - 6 - ALIGNMENT_ERROR) {
                leftDrivePower = -TURN_SPEED / 2;
                rightDrivePower = TURN_SPEED / 2;
            } else {
                leftDrivePower = -TURN_SPEED;
                rightDrivePower = TURN_SPEED;
            }
        } else {
            DriverStation.reportError("Alligned.", false);
            leftDrivePower = 0;
            rightDrivePower = 0;
            return true;
        }
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
    }

}
