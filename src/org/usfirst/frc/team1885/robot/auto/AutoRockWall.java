package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

public class AutoRockWall extends AutoCommand{
    private final double ROCK_SPEED_MAX = 0.8; // Subject to testing change
    private final double ROCK_SPEED_MED = 0.5; // Subject to testing change
    private final double ROCK_SPEED_MIN = 0.2; // Subject to testing change
    private SensorInputControlSRX sensorControl = SensorInputControlSRX
            .getInstance();
    private RobotControlWithSRX robotControl = RobotControlWithSRX
            .getInstance();
    private DrivetrainControl driveTrainControl = DrivetrainControl
            .getInstance();
    private AutoCrossedDefense crossedDefense;
    private double leftDriveSpeed;
    private double rightDriveSpeed; 
    
    public boolean init() {
        DrivetrainControl.getInstance().setControlMode(TalonControlMode.Speed);
        reset();
        leftDriveSpeed = driveTrainControl.getLeftDriveSpeed();
        rightDriveSpeed = driveTrainControl.getRightDriveSpeed();
        crossedDefense = new AutoCrossedDefense();
        return true;
    }
    
    public boolean execute() {
        double roll = sensorControl.getRoll();
        //DriverStation.reportError("\nRight side: " + leftDriveSpeed
                //+ " --- Left side: " + rightDriveSpeed, false);
        if (crossedDefense.execute()) {
            return true;
        }
            
        if (roll > 4.5) {
                leftDriveSpeed = ROCK_SPEED_MED + (roll/50) * (.3);
                rightDriveSpeed = ROCK_SPEED_MED + (roll/50) * (.3);
            }
         else if (roll < -4.5) {
            leftDriveSpeed = ROCK_SPEED_MED - (-roll/50) * (.3);
            rightDriveSpeed = ROCK_SPEED_MED - (-roll/50) * (.3);
        }
        else
        {
            leftDriveSpeed = driveTrainControl.getLeftDriveSpeed();
            rightDriveSpeed = driveTrainControl.getRightDriveSpeed();   
        }
        driveTrainControl.setLeftDriveSpeed(leftDriveSpeed);
        driveTrainControl.setRightDriveSpeed(rightDriveSpeed);
        return false;
    }
    
    @Override
    public boolean updateOutputs() {
        DrivetrainControl.getInstance().updateOutputs();
        return true;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }
}
