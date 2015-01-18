package org.usfirst.frc.team1885.robot;


import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically it 
 * contains the code necessary to operate a robot with tank drive.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
public class Robot extends SampleRobot 
{
    private int leftPort;
    private int rightPort;
    private final double diameter;
    private final double maxSpeed;
    private DriverInputControl joystickControl;
    private DrivetrainControl driveTrainControl;
    private RobotControl robotControl;
    public Robot() 
    {
        leftPort = 0;
        rightPort = 1;
        diameter = 4.0;
        maxSpeed = 15.0; 
    	this.joystickControl = DriverInputControl.getInstance();
    	this.driveTrainControl = new DrivetrainControl(diameter, maxSpeed);
    	this.driveTrainControl.addSpeed(1, 15.0);
    	this.robotControl = RobotControl.getInstance();
    }    
    /**
     * Runs the motors with tank steering.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) 
        {        	
//        	System.out.println( joystickControl.getLeftDrive() + " " + joystickControl.getRightDrive());
//        	System.out.println( driveTrainControl.getLeftDriveSpeed() + " " + driveTrainControl.getRightDriveSpeed());
        	driveTrainControl.update(joystickControl.getLeftDrive(), joystickControl.getRightDrive());
        	robotControl.updateDriveSpeed(driveTrainControl.getLeftDriveSpeed(), driveTrainControl.getRightDriveSpeed());
            Timer.delay(.005);		// wait for a motor update time
        }
    }

}
