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
    private final double ALIGNMENT_ERROR = .25;
    private final double MIN_SPEED = 0.2;
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
        P = 0.7;
        I = 0.01;
        D = 0;
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

        double difference = (yaw - targetDegree);

        if (Math.abs(difference) > Math.abs((360 - yaw) + targetDegree)) {
            difference = -((360 - yaw) + targetDegree);
        }

        if (Math.abs(difference) > Math.abs((360 - targetDegree) + yaw)) {
            difference = ((360 - targetDegree) + yaw);
        }

        rightDrivePower = leftDrivePower = pid.getPID(0, difference);

        DriverStation.reportError("\n Degree to turn : " + targetDegree
                + " --- Current yaw: " + sensorInputControl.getYaw()
                + "\n Pid Speed:: " + rightDrivePower + "\n difference:: " + difference, false);

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

        if (difference > ALIGNMENT_ERROR) {
            rightDrivePower = -rightDrivePower;
        } else if (difference < -ALIGNMENT_ERROR) {
            leftDrivePower = -leftDrivePower;
        } else {
            DriverStation.reportError("Alligned.", false);
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

}
