package org.usfirst.frc.team1885.robot;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.auto.AutoCommand;
import org.usfirst.frc.team1885.robot.auto.AutoTemplate;
import org.usfirst.frc.team1885.robot.auto.AutonomousRoutine;
import org.usfirst.frc.team1885.robot.config2016.RobotConfiguration;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.manipulator.UtilityArm;
import org.usfirst.frc.team1885.robot.modules.ActiveIntake;
import org.usfirst.frc.team1885.robot.modules.Shooter;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;
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
    private LinkedList<AutoCommand> commands;
    private long timeTracker = 0;
    private double delayTime = 1;// Input time in seconds

    //Output Control
    private RobotControlWithSRX robotControl;
    //InputControl
    private DriverInputControlSRX driverInputControl;
    private SensorInputControlSRX sensorInputControl;
    // Module Control
    private DrivetrainControl drivetrainControl;
    private ActiveIntake activeIntake;
    private Shooter shooter;
    private UtilityArm ultilityArm;
    

    private AutoTemplate activeTemplate;

    public Robot() {
        //Initialize Output Control
        robotControl = RobotControlWithSRX.getInstance();
        //Initialize Input Control
        driverInputControl = DriverInputControlSRX.getInstance();
        sensorInputControl = SensorInputControlSRX.getInstance();
        try {
            Timer.delay(0.5);
            //Configure Robot
            RobotConfiguration.configureRobot();
            //Initialize Sensor Values
            sensorInputControl.init();
        } catch (Exception e) {
            DriverStation.reportError("Robot - Error configuring Robot", false);
        }
        diameter = 9.0;
        //Initialize Modules
        drivetrainControl = DrivetrainControl.getInstance();
        activeIntake = ActiveIntake.getInstance();
        shooter = Shooter.getInstance();
        ultilityArm = UtilityArm.getInstance();
    }

    /**
     * Runs the motors with tank steering.
     */
    public void operatorControl() {

        while (isOperatorControl() && isEnabled()) {
            //Update Inputs
            sensorInputControl.update();
            driverInputControl.update();
            //Update Module Data
            drivetrainControl.update();
            activeIntake.update();
            shooter.update();
//            utilityArm.update
            //Update Module Outputs
            drivetrainControl.updateOutputs();
            activeIntake.updateOutputs();
            shooter.updateOutputs();
//            utilityArm.updateOutputs();
            Timer.delay(.005);
        }
    }

    public void autonomous() {
        AutonomousRoutine ar = new AutonomousRoutine(this);
        ar.execute();
    }
    
}
