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

    private final double P, I, D;
    private final double ALIGNMENT_ERROR = .5;
    private final double MIN_SPEED = 0.3;
    private double targetDegree;
    private PID pid;
    private SensorInputControlSRX sensorInputControl;
    private double rightDrivePower;
    private double leftDrivePower;

    public AutoAlign() {
        this(0);
    }

    public AutoAlign(double degree) {
        targetDegree = (degree < 0 ? 360 + degree : degree % 360);
        double scale = 180 / Math.abs(degree == 0 ? 1 : degree);
        P = 1 / scale;
        I = 0.001 / scale;
        D = 5 / scale;
        pid = new PID(P, I, D);
        pid.setScalingValue(targetDegree);
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

//        if (yaw < 0) {
//            yaw += 360;
//        }

        double difference = (yaw - targetDegree);

        rightDrivePower = pid.getPID(difference < -180 ? -360 : 0, difference);
        leftDrivePower = rightDrivePower * -1;

        DriverStation.reportError("\n Degree to turn : " + targetDegree
                + " --- Normalized yaw: " + yaw + "\n Pid Speed:: "
                + rightDrivePower + "\n difference:: " + difference, false);

        if (leftDrivePower > 0) {
            leftDrivePower = (leftDrivePower < MIN_SPEED ? MIN_SPEED
                    : leftDrivePower);
        } else if (leftDrivePower < 0) {
            leftDrivePower = (leftDrivePower > -MIN_SPEED ? -MIN_SPEED
                    : leftDrivePower);
        }

        if (rightDrivePower > 0) {
            rightDrivePower = (rightDrivePower < MIN_SPEED ? MIN_SPEED
                    : rightDrivePower);
        } else if (rightDrivePower < 0) {
            rightDrivePower = (rightDrivePower > -MIN_SPEED ? -MIN_SPEED
                    : rightDrivePower);
        }

        if (Math.abs((difference < 0 ? difference + 360 : difference)) < ALIGNMENT_ERROR) {
            DriverStation.reportError("\nAligned.", false);
            this.reset();
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
        pid.reset();
        leftDrivePower = 0;
        rightDrivePower = 0;
        DrivetrainControl.getInstance().setLeftDriveSpeed(0);
        DrivetrainControl.getInstance().setRightDriveSpeed(0);
    }

    public void setTargetDegree(double degree) {
        targetDegree = (degree < 0 ? 360 + degree : degree % 360);
        pid.setScalingValue(targetDegree);
    }
}