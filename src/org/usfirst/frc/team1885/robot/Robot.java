package org.usfirst.frc.team1885.robot;


import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.auto.AutoArc;
import org.usfirst.frc.team1885.robot.auto.AutoClaw;
import org.usfirst.frc.team1885.robot.auto.AutoCommand;
import org.usfirst.frc.team1885.robot.auto.AutoDriveForward;
import org.usfirst.frc.team1885.robot.auto.AutoToteLift;
import org.usfirst.frc.team1885.robot.auto.AutoTurn;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.comms.DataTelemetryService;
import org.usfirst.frc.team1885.robot.comms.RobotServer;
import org.usfirst.frc.team1885.robot.config2015.RobotConfiguration;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.manipulator.ClawControl;
import org.usfirst.frc.team1885.robot.modules.drivetrain.Alignment;
import org.usfirst.frc.team1885.robot.modules.drivetrain.BackupRoutine;
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
    private LinkedList<AutoCommand> commands;
    
    public Robot() {
    	try {
	        RobotConfiguration.configureRobot();
	        SensorInputControl.getInstance().getNAVX().zeroYaw();
    	} catch(Exception e) {
    		System.out.println("Robot - Error configuring Robot");
    		e.printStackTrace();
    	}
        diameter = 4.0;
        maxSpeed = 15.0; 
         
    	DrivetrainControl.getInstance().addSpeed(1, 15.0);
    	this.robotControl = RobotControl.getInstance();
    	this.recycleBinLift = RecycleBinLift.getInstance();
    	this.toteLift = ToteLift.getInstance();
    	
//    	RobotServer.getInstance().setup(4444);
//    	RobotServer.getInstance().startServer();
    	
    }    
    /**
     * Runs the motors with tank steering.
     */
    public void operatorControl() {
    	
    	SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).reset();
    	SensorInputControl.getInstance().getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).reset();
    	
    	boolean magnetState = false;
    	
    	DataTelemetryService telemetryService = new DataTelemetryService();
    	
        while (isOperatorControl() && isEnabled()) {
        	
//        	try {
//        		telemetryService.setDigitalInputs();
//        		telemetryService.setRelays();
//        		telemetryService.setSolenoids();
//        		telemetryService.setTalons();
//        		RobotServer.getInstance().send(telemetryService.getTm());
//        	} catch(Exception e) {
//        		e.printStackTrace();
//        	}
        	
//        	System.out.println(SensorInputControl.getInstance().getNAVX().getYaw360());
        	
        	DrivetrainControl.getInstance().update();
        	ClawControl.getInstance().updateClaw();
        	toteLift.updateLift();
        	recycleBinLift.updateLift();
        	Alignment.getInstance().update();
//        	System.out.println("Robot::tele - lidar: " + SensorInputControl.getInstance().getLidarSensor(SensorType.LIDAR).getDistance());
//        	BackupRoutine.getInstance().update();
        	
        	//robotControl.updateDriveSpeed(DrivetrainControl.getInstance().getLeftDriveSpeed(), DrivetrainControl.getInstance().getRightDriveSpeed());
        	recycleBinLift.updateOutputs();
        	toteLift.updateOutputs();
        	DrivetrainControl.getInstance().updateOutputs();
        	ClawControl.getInstance().updateOutputs();
        	Alignment.getInstance().updateOutputs();
            Timer.delay(.005);		// wait for a motor update time
        }
    }
    
    public void autonomous() {
    	commands = new LinkedList<AutoCommand>();
    	
    	autoOneBinOneTote();
    	
    	while(!commands.isEmpty() &&  isEnabled() && isAutonomous()) {
    		
    		AutoCommand currCommand = commands.peek();
    		
    		
    		if(currCommand.isInit()) {
	    		boolean commandState = currCommand.execute();
	    		currCommand.updateOutputs();
	    		if(commandState) {
	    			System.out.println("Finished command " + commands.size());
	    			commands.poll();
	    		}
    		} else {
    			currCommand.setInit(currCommand.init());
    		}
    	    		
    		Timer.delay(.005);
    	}
    	
    	DrivetrainControl.getInstance().update(0,0);
    	DrivetrainControl.getInstance().updateOutputs();
    }
    
    public void autoOneBinOneTote() {
    	commands.add(new AutoClaw(true, true, true)); //rotation, extension, pinch
    	commands.add(new AutoDriveForward(.5 *12, 1, 2));
    	commands.add(new AutoTurn(90, 1));
    	commands.add(new AutoDriveForward(2.0 *12, 3, 2));
    	commands.add(new AutoTurn(90, 1));
    	commands.add(new AutoDriveForward(3.5 *12, 1, 2));
    	commands.add(new AutoToteLift(1210, 10));
    	commands.add(new AutoDriveForward(-6 * 12, 1, 2));
    	commands.add(new AutoToteLift(-1210, 10));
//    	commands.add(new AutoTurn(180, 1));
    }
    public void autoOneBinThreeTotes() {
    	commands.add(new AutoToteLift(2 * 1210, 10));
    	commands.add(new AutoDriveForward(1 * 12, 1, 2));
    	commands.add(new AutoTurn(-60, 1));
    	commands.add(new AutoDriveForward(.5 * 12, 1, 2)); 
    	commands.add(new AutoToteLift(1 * 1210, 10));
    	commands.add(new AutoDriveForward(6 * 12, 1, 2));
    	commands.add(new AutoToteLift(1 * 1210, 10));
    	commands.add(new AutoDriveForward(6 * 12, 1, 2));
    	commands.add(new AutoToteLift(1 * 1210, 10));
    	commands.add(new AutoTurn(-90, 1));
    	commands.add(new AutoDriveForward(5 * 12, 1, 2));
    }
    public void autoSimple() {
        commands.add(new AutoToteLift(3 * 1210, 10));
        commands.add(new AutoDriveForward(1.5 * 12, 1, 2));
        commands.add(new AutoTurn(-55, 1));
        commands.add(new AutoDriveForward(.5 * 12, 1, 2));
//        commands.add(new AutoToteLift(-1 * 1210, 10));
        commands.add(new AutoToteLift(1 * 1210, 10));
        commands.add(new AutoTurn(-90, 1));
        commands.add(new AutoDriveForward(5 * 12, 1, 2));
        commands.add(new AutoToteLift(-1 * 1210, 10));
    }

}
