package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

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
        sensorInputControl = SensorInputControlSRX.getInstance();
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
        leftPow = DrivetrainControl.getInstance().getLeftDriveSpeed() * .5;
        rightPow = DrivetrainControl.getInstance().getRightDriveSpeed() * .5;
        DriverStation.reportError("\nLeft Turn:: " + leftPow + "\nRight Turn:: "
                + rightPow + "\n", false);
        boolean isDriveDisFinished = driveDis.execute();
        if (leftPow > 0) {
            leftPow = -leftPow
                    + DrivetrainControl.getInstance().getLeftDriveSpeed() * .75;
            rightPow = rightPow
                    + DrivetrainControl.getInstance().getRightDriveSpeed() * .75;
        } else {
            leftPow = leftPow
                    + DrivetrainControl.getInstance().getLeftDriveSpeed() * .75;
            rightPow = -rightPow
                    + DrivetrainControl.getInstance().getRightDriveSpeed() * .75;
        }

        DriverStation.reportError("\nDriveDis Left Power:: "
                + DrivetrainControl.getInstance().getLeftDriveSpeed()
                + "\nDriveDis Right Power:: "
                + DrivetrainControl.getInstance().getRightDriveSpeed() + "\n",
                false);
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
