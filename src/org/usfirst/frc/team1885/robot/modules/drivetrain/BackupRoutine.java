package org.usfirst.frc.team1885.robot.modules.drivetrain;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class BackupRoutine {
    private PID distanceControlLoop;
    private final double ERROR = 100;
    private final double minDistance = 50;
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
    	distanceFromTarget = SensorInputControl.getInstance().getLidarSensor(SensorType.LIDAR).getDistance();
    	
    	
    	double driveOutput = distanceControlLoop.getPID(minDistance, distanceFromTarget);
    	
    	System.out.println(distanceFromTarget + ", " + driveOutput);
    	
        if(driveOutput > .1) {
            leftDriveOutput = (driveOutput < DrivetrainControl.getInstance().getLeftDriveSpeed() ? DrivetrainControl.getInstance().getLeftDriveSpeed() : driveOutput);
            rightDriveOutput = (driveOutput < DrivetrainControl.getInstance().getRightDriveSpeed() ? DrivetrainControl.getInstance().getRightDriveSpeed() : driveOutput);
        } else {
            leftDriveOutput = DrivetrainControl.getInstance().getLeftDriveSpeed();
            rightDriveOutput = DrivetrainControl.getInstance().getRightDriveSpeed();
            reset();
        }
        //FIXME: inverted for forward...make configurable
        DrivetrainControl.getInstance().update(leftDriveOutput, rightDriveOutput);
    }
    public void reset() {
        distanceControlLoop.reset();
    }
	@Override
	public String toString() {
		return "BackupRoutine [distanceControlLoop=" + distanceControlLoop
				+ ", ERROR=" + ERROR + ", leftDriveOutput=" + leftDriveOutput
				+ ", rightDriveOutput=" + rightDriveOutput
				+ ", distanceFromTarget=" + distanceFromTarget + "]";
	}
}
