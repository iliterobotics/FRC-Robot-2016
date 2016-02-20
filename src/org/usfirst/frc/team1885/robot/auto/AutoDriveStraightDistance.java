package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AutoDriveStraightDistance extends AutoCommand {
    private SensorInputControlSRX sensorInputControl;

    private double disToTravel;
    private boolean doesStop;
    private double leftPow;
    private double rightPow;

    private double straightYaw;

    private AutoDriveDistance driveDis;
    private AutoAlign align;

    public AutoDriveStraightDistance(double distance, boolean doesStop) {
        disToTravel = distance;
        this.doesStop = doesStop;
    }

    public AutoDriveStraightDistance(double distance, boolean doesStop,
            double leftPow, double rightPow) {
        this(distance, doesStop);
        this.leftPow = leftPow;
        this.rightPow = rightPow;
    }

    @Override
    public boolean init() {
        if (leftPow == 0) {
            leftPow = DrivetrainControl.getInstance().getLeftDriveSpeed();
        }
        if (rightPow == 0) {
            rightPow = DrivetrainControl.getInstance().getRightDriveSpeed();
        }
        driveDis = new AutoDriveDistance(disToTravel, doesStop, leftPow,
                rightPow);
        align = new AutoAlign();
        straightYaw = sensorInputControl.getYaw();
        if (driveDis.init() && align.init()) {
            align.setTargetDegree(straightYaw);
            return true;
        }
        return false;
    }

    @Override
    public boolean execute() {
        boolean isAlignFinished = align.execute();
        leftPow = DrivetrainControl.getInstance().getLeftDriveSpeed();
        rightPow = DrivetrainControl.getInstance().getRightDriveSpeed();
        boolean isDriveDisFinished = driveDis.execute();
        leftPow = leftPow + DrivetrainControl.getInstance().getLeftDriveSpeed();
        rightPow = rightPow
                + DrivetrainControl.getInstance().getRightDriveSpeed();
        return isAlignFinished && isDriveDisFinished;
    }

    @Override
    public boolean updateOutputs() {
        RobotControlWithSRX.getInstance().updateDriveSpeed(leftPow, rightPow);
        return false;
    }

    @Override
    public void reset() {
        // No values to reset
    }

}
