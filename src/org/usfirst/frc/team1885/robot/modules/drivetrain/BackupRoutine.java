package org.usfirst.frc.team1885.robot.modules.drivetrain;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class BackupRoutine {
    private PID rightDistanceControlLoop;
    private PID leftDistanceControlLoop;
    private double distance;
    private double error;
    private double leftDriveOutput;
    private double rightDriveOutput; 
    private double distanceTraveled;
    // lidar output in centimeters, distance var input needs to match
    public BackupRoutine(double d, double e) {
        rightDistanceControlLoop = new PID(0.01, 0.00001, 0);
        leftDistanceControlLoop = new PID(0.01, 0.00001, 0);
        distance = d;
        error = e;
        reset();
    }
    public void update() {
        System.out.println("AutoDriveFwd::[left dist, right dist] " + distanceTraveled + ", " + distanceTraveled);
        
        if (Math.abs(distanceTraveled  - distance) <= error && Math.abs(distanceTraveled  - distance) <= error ) {
            this.reset();
        }
        
        distanceTraveled = SensorInputControl.getInstance().getLidarSensor(SensorType.LIDAR).getDistance();
        if(Math.abs(distanceTraveled  - distance) > error) {
            leftDriveOutput = leftDistanceControlLoop.getPID(distance, distanceTraveled);
            rightDriveOutput = rightDistanceControlLoop.getPID(distance, distanceTraveled);
        } else {
            leftDistanceControlLoop.reset();
            leftDriveOutput = 0;
            rightDistanceControlLoop.reset();
            rightDriveOutput = 0;
        }
        
        System.out.println("AutoDriveFwd::[left speed, right speed] " + leftDriveOutput + ", " + rightDriveOutput);
        
        RobotControl.getInstance().updateDriveSpeed(-leftDriveOutput, -rightDriveOutput);
    }
    public void reset() {
        rightDistanceControlLoop.reset();
        leftDistanceControlLoop.reset();
        RobotControl.getInstance().updateDriveSpeed(0, 0);
    }
}
