package org.usfirst.frc.team1885.robot;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.auto.AutoCommand;
import org.usfirst.frc.team1885.robot.auto.AutoTemplate;
import org.usfirst.frc.team1885.robot.auto.AutonomousRoutine;
import org.usfirst.frc.team1885.robot.config2016.RobotConfiguration;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.Shooter;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

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
    private LinkedList<AutoCommand> commands;
    private long timeTracker = 0;
    private double delayTime = 1;// Input time in seconds

    private RobotControlWithSRX robotControlWithSRX;
    private DriverInputControlSRX driverInputControl;
    private SensorInputControlSRX sensorInputControl;

    private AutoTemplate activeTemplate;

    public Robot() {
        this.robotControlWithSRX = RobotControlWithSRX.getInstance();
        this.driverInputControl = DriverInputControlSRX.getInstance();
        this.sensorInputControl = SensorInputControlSRX.getInstance();
        try {
            RobotConfiguration.configureRobot();
            sensorInputControl.init();
        } catch (Exception e) {
            System.out.println("Robot - Error configuring Robot");
            e.printStackTrace();
        }
        diameter = 4.0;
        maxSpeed = 15.0;

        DrivetrainControl.getInstance().addSpeed(1, 15.0);

    }

    /**
     * Runs the motors with tank steering.
     */
    public void operatorControl() {

        // TODO: FIX THIS NPE.
        // SensorInputControl.getInstance()
        // .getEncoder(SensorType.DRIVE_TRAIN_LEFT_ENCODER).reset();
        // SensorInputControl.getInstance()
        // .getEncoder(SensorType.DRIVE_TRAIN_RIGHT_ENCODER).reset();

        while (isOperatorControl() && isEnabled()) {
            sensorInputControl.update();
//            driverInputControl.update();
            Shooter.getInstance().update();
          //UtilityArm.getInstance().update();
            Timer.delay(.005);
        }

        // System.out.println(SensorInputControl.getInstance().getEncoderTicks(SensorType.DRIVE_TRAIN_RIGHT_ENCODER)
        // + " "
        // +
        // SensorInputControl.getInstance().getEncoderTicks(SensorType.DRIVE_TRAIN_LEFT_ENCODER)
        // + " " +
        // SensorInputControl.getInstance().getEncoderTicks(SensorType.TOTE_ENCODER)
        // + " " +
        // SensorInputControl.getInstance().isActive(SensorType.RECYCLE_BIN_LOWER_LIMIT)
        // + " " +
        // SensorInputControl.getInstance().isActive(SensorType.RECYCLE_BIN_UPPER_LIMIT));
        // if (System.currentTimeMillis() - timeTracker >= (delayTime * 1000)) {
        // timeTracker = System.currentTimeMillis();
        // try {
        // robotStatusService.update();
        // RobotServer.getInstance().send(robotStatusService.getTm());
        // } catch (Exception e) {
        // e.printStackTrace();
        // }
        //
        //// System.out.println(SensorInputControl.getInstance().getNAVX()
        //// .getYaw360());
        // }
        /*
         * if((DriverInputControl.getInstance().getButton(
         * RobotButtonType.CANCEL_AUTOMATION))) { this.activeTemplate = null; }
         * 
         * // if((DriverInputControl.getInstance().getButton( //
         * RobotButtonType.AUTOMATE_2_TOTES)) || this.activeTemplate != null) {
         * // // if(this.activeTemplate == null) { // this.activeTemplate =
         * AutoTemplate.automate2Totes(); // } // //
         * if(this.activeTemplate.execute()) { // this.activeTemplate = null; //
         * } // // } else { this.activeTemplate = null;
         * DrivetrainControl.getInstance().update();
         * ActiveIntake.getInstance().update(); System.out.println(
         * "Updated active intake" ); // System.out.println(
         * "Robot::tele - lidar: " + //
         * SensorInputControl.getInstance().getLidarSensor(SensorType.LIDAR).
         * getDistance()); // BackupRoutine.getInstance().update();
         * 
         * // robotControl.updateDriveSpeed(DrivetrainControl.getInstance().
         * getLeftDriveSpeed(), //
         * DrivetrainControl.getInstance().getRightDriveSpeed());
         * DrivetrainControl.getInstance().updateOutputs();
         * ActiveIntake.getInstance().updateOutputs(); System.out.println(
         * "Updated active intake outputs" ); } Timer.delay(.005); // wait for a
         * motor update time }
         */
    }

    public void autonomous() {
        AutonomousRoutine ar = new AutonomousRoutine(this);
//        sensorrx.resetEncoder(SensorType.LEFT_ENCODER);
//        sensorrx.resetEncoder(SensorType.RIGHT_ENCODER);
        ar.execute();
    }
    
}
