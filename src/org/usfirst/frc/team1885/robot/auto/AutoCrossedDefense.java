package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

/**
 * Waits until the robot has crossed over a defense. This is determined by if
 * the robot has been level with the ground for a certain amount of time.
 * 
 * @author ILITE Robotics
 * @version <2/13/2016>
 */
public class AutoCrossedDefense extends AutoCommand {

    private final static double ERROR = 2.5; // Margin of error for Pitch and Roll
    private final double FLAT_PITCH;// Initial Pitch of robot on creation of class
    private final double FLAT_ROLL;// Initial Roll of robot on creation of class
    private final static double WAIT_TIME = 0.250;// Time (in seconds) the robot must be 'flat' to be considered on flat ground

    private SensorInputControlSRX sensorInputControl;
    private long startTime;
    private double leftDriveSpeed;
    private double rightDriveSpeed;

    public AutoCrossedDefense() {
        sensorInputControl = SensorInputControlSRX.getInstance();
        FLAT_ROLL = sensorInputControl.getInitRoll();
        FLAT_PITCH = sensorInputControl.getInitPitch();
    }

    @Override
    public boolean init() {
        DrivetrainControl.getInstance().setControlMode(TalonControlMode.Speed);
        
        startTime = System.currentTimeMillis();
        leftDriveSpeed = DrivetrainControl.getInstance().getLeftDriveSpeed();
        rightDriveSpeed = DrivetrainControl.getInstance().getRightDriveSpeed();
        
        return true;
    }

    @Override
    public boolean execute() {
        double currentRoll = sensorInputControl.getNavX().getRoll();
        double currentPitch = sensorInputControl.getNavX().getPitch();

        boolean isAlignedRoll = currentRoll <= FLAT_ROLL + ERROR && currentRoll >= FLAT_ROLL - ERROR;

        boolean isAlignedPitch = currentPitch <= FLAT_PITCH + ERROR && currentPitch >= FLAT_PITCH - ERROR;

        if (isAlignedPitch && isAlignedRoll) {
            if (System.currentTimeMillis() - startTime > WAIT_TIME * 1000) {
                leftDriveSpeed = rightDriveSpeed = 0;
                return true;
            }
        } else {
            startTime = System.currentTimeMillis();
        }
        if(timeOut()){
            return true;
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveSpeed);
        DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveSpeed);
        DrivetrainControl.getInstance().updateOutputs();
        return false;
    }

    @Override
    public void reset() {}
}
