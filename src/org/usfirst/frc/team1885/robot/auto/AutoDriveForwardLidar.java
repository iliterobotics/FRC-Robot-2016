package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AutoDriveForwardLidar extends AutoCommand {

    private PID distanceControlLoop;
    private double stopDistance;
    private double error;
    private double driveOutput;
    private double distanceFromTarget;

    public AutoDriveForwardLidar(double d, double e) {
        distanceControlLoop = new PID(0.01, 0.00001, 0);
        stopDistance = d;
        error = e;
        reset();
    }
    public boolean execute() {
        System.out.println("AutoDriveFwd::[dist] " + distanceFromTarget);

        if (Math.abs(distanceFromTarget - stopDistance) <= error) {
            this.reset();
            return true;
        }

        if (Math.abs(distanceFromTarget - stopDistance) > error) {
            driveOutput = distanceControlLoop.getPID(stopDistance,
                    distanceFromTarget);
        } else {
            distanceControlLoop.reset();
            driveOutput = 0;
        }

        DrivetrainControl.getInstance().update(-driveOutput, -driveOutput);
        return false;
    }
    public void reset() {
        distanceControlLoop.reset();
        RobotControlWithSRX.getInstance().updateDriveSpeed(0, 0);
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
