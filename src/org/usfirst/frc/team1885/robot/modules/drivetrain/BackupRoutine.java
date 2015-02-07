package org.usfirst.frc.team1885.robot.modules.drivetrain;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class BackupRoutine {
    private PID distanceControlLoop;
    private final double ERROR = 100;
    private double leftDriveOutput;
    private double rightDriveOutput;
    private double distanceFromTarget;
    private static BackupRoutine instance;
    // lidar output in centimeters, distance var input needs to match
    protected BackupRoutine() {
        distanceControlLoop = new PID(0.01, 0.00001, 0);
        reset();
    }
    public static BackupRoutine getInstance(){
        if( instance == null ){
            instance = new BackupRoutine();
        }
        return instance;
    }
    public void update() {
        if(distanceFromTarget < ERROR) {
            distanceFromTarget = SensorInputControl.getInstance().getLidarSensor(SensorType.LIDAR).getDistance();
            leftDriveOutput = distanceControlLoop.getPID(ERROR, distanceFromTarget);
            rightDriveOutput = leftDriveOutput;
        } else {
            leftDriveOutput = DrivetrainControl.getInstance().getLeftDriveSpeed();
            rightDriveOutput = DrivetrainControl.getInstance().getRightDriveSpeed();
            reset();
        }
        DrivetrainControl.getInstance().update(leftDriveOutput, rightDriveOutput);
    }
    public void reset() {
        distanceControlLoop.reset();
    }
}
