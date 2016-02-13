package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AutoRamparts extends AutoCommand{
    //Made by Aaron
    private final double YAW_ZONE = 5.0;
    private SensorInputControlSRX sensorControl= SensorInputControlSRX.getInstance();
    private RobotControlWithSRX robotControl = RobotControlWithSRX.getInstance();
    private DrivetrainControl driveTrainControl = DrivetrainControl.getInstance();
    private double leftDriveSpeed;
    private double rightDriveSpeed;
    @Override
    public boolean init() {
        reset();
        leftDriveSpeed = driveTrainControl.getLeftDriveSpeed();
        rightDriveSpeed = driveTrainControl.getRightDriveSpeed();
        return true;
    }

    @Override
    public boolean execute() {
        double yaw = sensorControl.getYaw();
        if((new AutoCrossedDefense()).execute())
        {
            return true;
        }
        if(this.inZone(yaw))
        {
            leftDriveSpeed = rightDriveSpeed = AutonomousRoutine.RAMPART_SPEED_MAX;
        }
        else if(yaw > YAW_ZONE)
        {
            if(driveTrainControl.getLeftDriveSpeed() == AutonomousRoutine.RAMPART_SPEED_MAX)
            {
                leftDriveSpeed = AutonomousRoutine.RAMPART_SPEED_MIN;
            }
            rightDriveSpeed = AutonomousRoutine.RAMPART_SPEED_MAX;
        }
        else if(yaw < -YAW_ZONE)
        {
            if(driveTrainControl.getRightDriveSpeed() == AutonomousRoutine.RAMPART_SPEED_MAX)
            {
                rightDriveSpeed = AutonomousRoutine.RAMPART_SPEED_MIN;
            }
            leftDriveSpeed = AutonomousRoutine.RAMPART_SPEED_MAX;
        }
        driveTrainControl.setLeftDriveSpeed(leftDriveSpeed);
        driveTrainControl.setRightDriveSpeed(rightDriveSpeed);
        return false;
    }

    @Override
    public boolean updateOutputs() {
        robotControl.updateDriveSpeed(leftDriveSpeed, rightDriveSpeed);
        return true;
    }

    @Override
    public void reset() {
        robotControl.updateDriveSpeed(0, 0);
    }

    public boolean inZone(double input)
    {
        if(input < YAW_ZONE && input > -YAW_ZONE)
        {
            return true;
        }
        return false;
    }
}
