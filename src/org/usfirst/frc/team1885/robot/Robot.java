package org.usfirst.frc.team1885.robot;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.auto.AutoCanGrabber;
import org.usfirst.frc.team1885.robot.auto.AutoClaw;
import org.usfirst.frc.team1885.robot.auto.AutoCommand;
import org.usfirst.frc.team1885.robot.auto.AutoDriveForward;
import org.usfirst.frc.team1885.robot.auto.AutoTemplate;
import org.usfirst.frc.team1885.robot.auto.AutoToteLift;
import org.usfirst.frc.team1885.robot.auto.AutoTurn;
import org.usfirst.frc.team1885.robot.auto.AutoWait;
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
				// System.out.println("Robot::tele - lidar: " +
				// SensorInputControl.getInstance().getLidarSensor(SensorType.LIDAR).getDistance());
				// BackupRoutine.getInstance().update();
	
				// robotControl.updateDriveSpeed(DrivetrainControl.getInstance().getLeftDriveSpeed(),
				// DrivetrainControl.getInstance().getRightDriveSpeed());
				recycleBinLift.updateOutputs();
				toteLift.updateOutputs();
				DrivetrainControl.getInstance().updateOutputs();
				ClawControl.getInstance().updateOutputs();
				Alignment.getInstance().updateOutputs();
				CanGrabber.getInstance().updateOutputs();
			}
			Timer.delay(.005); // wait for a motor update time
		}
	}

	public void autonomous() {
		commands = new LinkedList<AutoCommand>();

		autoOneTote();
//		autoCanGrabber();
		while (!commands.isEmpty() && isEnabled() && isAutonomous()) {

			AutoCommand currCommand = commands.peek();

			if (currCommand.isInit()) {
				boolean commandState = currCommand.execute();
				currCommand.updateOutputs();
				if (commandState) {
					System.out.println("Finished command " + commands.size());
					commands.poll();
				}
			} else {
				currCommand.setInit(currCommand.init());
			}

			Timer.delay(.005);
		}

		DrivetrainControl.getInstance().update(0, 0);
		DrivetrainControl.getInstance().updateOutputs();
	}
	
	public void autoCanGrabber() {
		//Moves forward and grabs the cans moves backwards
		commands.add(new AutoCanGrabber(true));
		commands.add(new AutoDriveForward(.75*12, 1, 2));
		commands.add(new AutoWait(1000));
		commands.add(new AutoDriveForward(-6*12, 1, 2));
//		commands.add(new AutoCanGrabber(false));
		
	}
	
	public void autoOneTote() {
		// Picks up one tote, drives backwards to autozone, drops tote
		commands.add(new AutoClaw(false, false, true));
		commands.add(new AutoToteLift(1210, 10));
		commands.add(new AutoToteLift(1210, 10));
		commands.add(new AutoToteLift(1210, 10));
		commands.add(new AutoDriveForward(-86, 1, 2));
		// commands.add(new AutoToteLift(-1210, 10));
	}
	
	public void autoOneCan() {
		// Picks up one tote, drives backwards to autozone, drops tote
		commands.add(new AutoClaw(false, false, true));
		commands.add(new AutoDriveForward(3 * 12, 1, 2));
		// commands.add(new AutoToteLift(-1210, 10));
	}

	public void autoOneToteTwoBin() {
		// Picks up one tote, pivots and pinches the bin, drives forward and
		// lifts the bin, turn 90 degrees and drive to autozone
		commands.add(new AutoToteLift(1 * 1210, 10));
		commands.add(new AutoTurn(-45, 5));
		commands.add(new AutoDriveForward(1 * 12, 1, 2));
		commands.add(new AutoTurn(-45, 5));
		commands.add(new AutoClaw(false, false, true));
		commands.add(new AutoDriveForward(6 * 12, 1, 2));
		commands.add(new AutoToteLift(3 * 1210, 10));
		commands.add(new AutoTurn(90, 5));
		commands.add(new AutoDriveForward(7 * 12, 1, 2));

	}

	public void autoOneToteOneBin() {
		// Lifts up tote, pivot and lift up bin, drive backwards to autozone
		// commands.add(new AutoDriveForward(3, 1, 2));
		commands.add(new AutoToteLift(1 * 1190, 10));
		commands.add(new AutoToteLift(2 * 1210, 10));
		commands.add(new AutoTurn(48, 5));
		commands.add(new AutoDriveForward(10, 1, 2));
		commands.add(new AutoToteLift(1 * 1210, 10));

		// commands.add(new AutoToteLift(3*1210, 10));
		commands.add(new AutoTurn(-45, 5));
		commands.add(new AutoDriveForward(-7 * 12, 1, 2));
		// commands.add(new AutoToteLift(-3 * 1210, 10));
	}

	public void autoPushOneBinOneTote() {
		// Pushes the tote out of the way, lifts bin, lifts tote, drives
		// backwards to autozone, drops totes
		// Pushes one bin out of the way, pivots and lifts
		commands.add(new AutoDriveForward(30, 1, 2));
		commands.add(new AutoDriveForward(-6, 1, 2));
		commands.add(new AutoTurn(90, 5));

		commands.add(new AutoCommand() {

			AutoDriveForward driveFwd = new AutoDriveForward(10, 1, 2);
			AutoToteLift toteLift = new AutoToteLift(3 * 1210, 10);

			@Override
			public boolean init() {
				// TODO Auto-generated method stub
				return driveFwd.init() && toteLift.init();
			}

			@Override
			public boolean execute() {
				return driveFwd.execute() && toteLift.execute();
			}

			@Override
			public boolean updateOutputs() {
				return driveFwd.updateOutputs() && toteLift.updateOutputs();
			}

			@Override
			public void reset() {
				driveFwd.reset();
				toteLift.reset();
			}

		});
		commands.add(new AutoDriveForward(-5, 1, 2));
		commands.add(new AutoTurn(-90, 5));
		commands.add(new AutoDriveForward(18, 1, 2));
		commands.add(new AutoWait(250));
		commands.add(new AutoDriveForward(3, 1, 2));
		commands.add(new AutoToteLift(1 * 1210, 10));
		commands.add(new AutoDriveForward(-7 * 12, 1, 2));
		commands.add(new AutoToteLift(-1 * 1210, 10));

	}

	public void autoOneBinOneTote() {
		// Pinches the bin, lifts the tote
		commands.add(new AutoClaw(false, false, true)); // rotation, extension,
														// pinch
		commands.add(new AutoDriveForward(.5 * 12, 1, 2));
		commands.add(new AutoTurn(90, 1));
		commands.add(new AutoDriveForward(2.0 * 12, 3, 2));
		commands.add(new AutoTurn(90, 1));
		commands.add(new AutoDriveForward(3.5 * 12, 1, 2));
		commands.add(new AutoToteLift(1210, 10));
		commands.add(new AutoDriveForward(-6 * 12, 1, 2));
		commands.add(new AutoToteLift(-1210, 10));
		// commands.add(new AutoTurn(180, 1));
	}

	public void autoOneBinThreeTotes() {
		//
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
		// commands.add(new AutoToteLift(-1 * 1210, 10));
		commands.add(new AutoToteLift(1 * 1210, 10));
		commands.add(new AutoTurn(-90, 1));
		commands.add(new AutoDriveForward(5 * 12, 1, 2));
		commands.add(new AutoToteLift(-1 * 1210, 10));
	}
}
