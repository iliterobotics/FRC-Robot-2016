package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.config2016.RobotConfiguration;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AutoRamparts extends AutoCommand{
    //Made by Aaron
    private SensorInputControlSRX sensorSRX= SensorInputControlSRX.getInstance();
    private RobotControlWithSRX rcsrx = RobotControlWithSRX.getInstance();
    private DrivetrainControl dtc = DrivetrainControl.getInstance();
    private double leftSpeed;
    private double rightSpeed;
    @Override
    public boolean init() {
        reset();
        leftSpeed = dtc.getLeftDriveSpeed();
        rightSpeed = dtc.getRightDriveSpeed();
        return true;
    }

    @Override
    public boolean execute() {
        // Drive forward and once BusVoltage dips drop right side power so that it goes over evenly
        // After we somehow figure out when we are done we stop the robot possibly on a time sensitive gyroscopic readings
        double yaw = sensorSRX.getYaw();
        double pitch = sensorSRX.getPitch();
        double roll = sensorSRX.getRoll();
        if(inZone(yaw) && inZone(pitch) && inZone(roll))
        {
            return true;
        }
        if(this.inZone(yaw))
        {
            leftSpeed = rightSpeed = RobotConfiguration.maxSpeed;
        }
        else if(yaw > 10)
        {
            if(dtc.getLeftDriveSpeed() == RobotConfiguration.maxSpeed)
            {
                leftSpeed = .6;
                rightSpeed = RobotConfiguration.maxSpeed;
            }
        }
        else if(yaw < -10)
        {
            if(dtc.getRightDriveSpeed() == RobotConfiguration.maxSpeed)
            {
                rightSpeed = .6;
                leftSpeed = RobotConfiguration.maxSpeed;
            }
        }
        rcsrx.updateDriveSpeed(leftSpeed, rightSpeed);
        return false;
    }

    @Override
    public boolean updateOutputs() {

        return true;
    }

    @Override
    public void reset() {
    }

    public boolean inZone(double input)
    {
        if(input < 10 && input > -10)
        {
            return true;
        }
        return false;
    }
}
