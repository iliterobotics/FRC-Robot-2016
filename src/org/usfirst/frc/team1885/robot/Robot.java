package org.usfirst.frc.team1885.robot;


import org.usfirst.frc.team1885.robot.comms.RobotServer;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends SampleRobot 
{
    private int leftPort;
    private int rightPort;
    private DriverInputControl joystickControl;
    private DrivetrainControl driveTrainControl;
    private RobotControl robotControl;
    private RobotServer robotServer;
    
    public Robot() 
    {
        leftPort = 0;
        rightPort = 1;
    	this.joystickControl = new DriverInputControl(leftPort,rightPort);
    	this.driveTrainControl = new DrivetrainControl();
    	this.robotControl = new RobotControl();
    	
    	this.robotServer = new RobotServer();
    	this.robotServer.setup(4444);
    	if(this.robotServer.startServer()) {
    		System.out.println("Robot::Constructor - Successfully started data server!");
    	}
    }    
    /**
     * Runs the motors with tank steering.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) 
        {        	
        	driveTrainControl.update(joystickControl.getLeftDrive(), joystickControl.getRightDrive());
        	robotControl.updateDriveSpeed(driveTrainControl.getLeftDriveSpeed(), driveTrainControl.getRightDriveSpeed());
            Timer.delay(.005);		// wait for a motor update time
        }
    }

}
