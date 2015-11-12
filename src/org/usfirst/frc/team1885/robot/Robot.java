package org.usfirst.frc.team1885.robot;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.auto.AutoCommand;
import org.usfirst.frc.team1885.robot.auto.AutoTemplate;
import org.usfirst.frc.team1885.robot.auto.AutonomousRoutine;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.comms.RobotServer;
import org.usfirst.frc.team1885.robot.comms.RobotStatusService;
import org.usfirst.frc.team1885.robot.config2015.RobotConfiguration;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.manipulator.ClawControl;
import org.usfirst.frc.team1885.robot.modules.drivetrain.Alignment;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.modules.lift.ActiveIntake;
import org.usfirst.frc.team1885.robot.modules.lift.CanGrabber;
import org.usfirst.frc.team1885.robot.modules.lift.RecycleBinLift;
import org.usfirst.frc.team1885.robot.modules.lift.ToteLift;
import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */

public class Robot extends SampleRobot {
	private final double diameter;
	private final double maxSpeed;
	private RobotControl robotControl;
	private RecycleBinLift recycleBinLift;
	private ToteLift toteLift;
	private LinkedList<AutoCommand> commands;
	private long timeTracker = 0;
	private double delayTime = 1;// Input time in seconds
	
	private AutoTemplate activeTemplate;

	public Robot() {
		try {
			RobotConfiguration.configureRobot();
			SensorInputControl.getInstance().getNAVX().zeroYaw();
		} catch (Exception e) {
			System.out.println("Robot - Error configuring Robot");
			e.printStackTrace();
		}
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

		SensorInputControl.getInstance()
				.getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).reset();
		SensorInputControl.getInstance()
				.getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).reset();

				
		
		
		boolean magnetState = false;

		RobotStatusService robotStatusService = new RobotStatusService();

		while (isOperatorControl() && isEnabled()) {
			
//			System.out.println(SensorInputControl.getInstance().getEncoderTicks(SensorType.DRIVE_TRAIN_RIGHT_ENCODER) + " " 
//					 + SensorInputControl.getInstance().getEncoderTicks(SensorType.DRIVE_TRAIN_LEFT_ENCODER) + " " +
//					SensorInputControl.getInstance().getEncoderTicks(SensorType.TOTE_ENCODER) + " " + 
//					SensorInputControl.getInstance().isActive(SensorType.RECYCLE_BIN_LOWER_LIMIT) + " " +
//					SensorInputControl.getInstance().isActive(SensorType.RECYCLE_BIN_UPPER_LIMIT));
//			if (System.currentTimeMillis() - timeTracker >= (delayTime * 1000)) {
//				timeTracker = System.currentTimeMillis();
//				try {
//					robotStatusService.update();
//					RobotServer.getInstance().send(robotStatusService.getTm());
//				} catch (Exception e) {
//					e.printStackTrace();
//				}
//
////				System.out.println(SensorInputControl.getInstance().getNAVX()
////						.getYaw360());
//			}
			
			if((DriverInputControl.getInstance().getButton(
				RobotButtonType.CANCEL_AUTOMATION))) {
				this.activeTemplate = null;
			}
			
//			if((DriverInputControl.getInstance().getButton(
//				RobotButtonType.AUTOMATE_2_TOTES)) || this.activeTemplate != null) {
//				
//				if(this.activeTemplate == null) {
//					this.activeTemplate = AutoTemplate.automate2Totes();
//				}
//				
//				if(this.activeTemplate.execute()) {
//					this.activeTemplate = null;
//				}
//				
//			}
			else
			{
				this.activeTemplate = null;
				DrivetrainControl.getInstance().update();
				ClawControl.getInstance().updateClaw();
				toteLift.updateLift();
				recycleBinLift.updateLift();
				Alignment.getInstance().update();
				CanGrabber.getInstance().update();
				ActiveIntake.getInstance().update();
				// System.out.println("Robot::tele - lidar: " +
				// SensorInputControl.getInstance().getLidarSensor(SensorType.LIDAR).getDistance());
				// BackupRoutine.getInstance().update();
	
				// robotControl.updateDriveSpeed(DrivetrainControl.getInstance().getLeftDriveSpeed(),
				// DrivetrainControl.getInstance().getRightDriveSpeed());
				recycleBinLift.updateOutputs();
				toteLift.updateOutputs();
				DrivetrainControl.getInstance().updateOutputs();
				ClawControl.getInstance().updateOutputs();
				ActiveIntake.getInstance().updateOutputs();
				Alignment.getInstance().updateOutputs();
				CanGrabber.getInstance().updateOutputs();
			}
			Timer.delay(.005); // wait for a motor update time
		}
	}

	public void autonomous() {
	    AutonomousRoutine ar = new AutonomousRoutine(this);
	    ar.autoOneTote();
	    ar.execute();
//		autoCanGrabber();		
	}
}
