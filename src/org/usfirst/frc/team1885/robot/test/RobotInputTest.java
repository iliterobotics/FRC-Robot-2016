package org.usfirst.frc.team1885.robot.test;


import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.comms.RobotServer;
import org.usfirst.frc.team1885.robot.comms.RobotServerEvent;
import org.usfirst.frc.team1885.robot.comms.RobotServerListener;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

public class RobotInputTest extends SampleRobot implements RobotServerListener 
{
    private int leftPort;
    private int rightPort;
    private final double diameter;
    private final double maxSpeed;
    private DriverInputControl joystickControl;
    private DrivetrainControl driveTrainControl;
    private RobotControl robotControl;
    private RobotServer robotServer;
    public RobotInputTest() 
    {
    	
        leftPort = 0;
        rightPort = 1;
        diameter = 4.0;
        maxSpeed = 15.0; 
    	this.joystickControl = DriverInputControl.getInstance();
    	this.joystickControl.addJoystick(RobotJoystickType.LEFT_DRIVE, leftPort);
    	this.joystickControl.addJoystick(RobotJoystickType.RIGHT_DRIVE, rightPort);
    	this.driveTrainControl = new DrivetrainControl(diameter, maxSpeed);
    	this.robotControl = RobotControl.getInstance();
    	
    	this.robotServer = RobotServer.getInstance();
    	this.robotServer.setup(4444);
    	this.robotServer.addListener( this );
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
    public void receivedServerEvent( RobotServerEvent event ) {
    	System.out.println( event.getMessage() );
    }

}
