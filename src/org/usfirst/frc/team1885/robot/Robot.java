package org.usfirst.frc.team1885.robot;


import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.auto.AutoCommand;
import org.usfirst.frc.team1885.robot.auto.AutoDriveForward;
import org.usfirst.frc.team1885.robot.auto.AutoWait;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.comms.RobotServer;
import org.usfirst.frc.team1885.robot.comms.TelemetryMessage;
import org.usfirst.frc.team1885.robot.config2015.RobotConfiguration;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.modules.lift.RecycleBinLift;
import org.usfirst.frc.team1885.robot.modules.lift.ToteLift;
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
    private final double diameter;
    private final double maxSpeed;
    private RobotControl robotControl;
    private RecycleBinLift recycleBinLift;
    private ToteLift toteLift;
    
    public Robot() {
        RobotConfiguration.configureRobot();
        diameter = 4.0;
        maxSpeed = 15.0; 

    	DrivetrainControl.getInstance().addSpeed(1, 15.0);
    	this.robotControl = RobotControl.getInstance();
    	this.recycleBinLift = RecycleBinLift.getInstance();
    	this.toteLift = ToteLift.getInstance();
    	
    	RobotServer.getInstance().setup(4444);
    	RobotServer.getInstance().startServer();
    	
    }    
    /**
     * Runs the motors with tank steering.
     */
    public void operatorControl() {
    	
    	SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).reset();
    	SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).reset();
    	
        while (isOperatorControl() && isEnabled()) {
        	
        	try {
        		RobotServer.getInstance().send(new TelemetryMessage());
        	} catch(Exception e) {
        		e.printStackTrace();
        	}
        	
        	System.out.println( "Robot::operatorControl - " + SensorInputControl.getInstance().getEncoderTicks(SensorType.DRIVE_TRAIN_LEFT_ENCODER) + ", " + SensorInputControl.getInstance().getEncoderTicks(SensorType.DRIVE_TRAIN_RIGHT_ENCODER));
        	driveTrainControl.update();
//        	System.out.println( driveTrainControl.getLeftDriveSpeed() + " " + driveTrainControl.getRightDriveSpeed());
        	robotControl.updateDriveSpeed(driveTrainControl.getLeftDriveSpeed(), driveTrainControl.getRightDriveSpeed());
        	recycleBinLift.updateLift();
        	toteLift.updateLift();
            Timer.delay(.005);		// wait for a motor update time
        }
    }
    
    public void autonomous() {
    	LinkedList<AutoCommand> commands;
    	commands = new LinkedList<AutoCommand>();
    	
    	commands.add(new AutoDriveForward(3.0*12,6.0));
    	commands.add(new AutoWait(5000.0));
    	commands.add(new AutoDriveForward(3.0*12,6.0));
    	commands.add(new AutoWait(5000.0));
    	commands.add(new AutoDriveForward(3.0*12,6.0));
    	commands.add(new AutoWait(5000.0));
//    	commands.add(new AutoTurn(90.0, .1));
    	
    	System.out.println("Starting Auto with " + commands.size() + " states");
    	
    	while(!commands.isEmpty() &&  isEnabled() && isAutonomous()) {
    		boolean commandState = commands.peek().execute();
    		if(commandState) {
    			System.out.println("Finished command " + commands.size());
    			commands.poll();
    		} else {
    			System.out.println("Executing command " + commands.size());
    		}
    		Timer.delay(.005);
    	}
    }

}
