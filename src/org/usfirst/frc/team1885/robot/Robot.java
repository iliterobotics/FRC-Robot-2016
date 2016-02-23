package org.usfirst.frc.team1885.robot;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.auto.AutoCommand;
import org.usfirst.frc.team1885.robot.auto.AutoTemplate;
import org.usfirst.frc.team1885.robot.auto.AutonomousRoutine;
import org.usfirst.frc.team1885.robot.config2016.RobotConfiguration;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.modules.ModuleControl;
import org.usfirst.frc.team1885.robot.modules.Shooter;
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
    private LinkedList<AutoCommand> commands;
    private long timeTracker = 0;
    private double delayTime = 1;// Input time in seconds

    //Output Control
    private RobotControlWithSRX robotControl;
    //InputControl
    private DriverInputControlSRX driverInputControl;
    private SensorInputControlSRX sensorInputControl;
    // Module Control
    private Module[] modules;

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
            Shooter.getInstance().init();
            DriverStation.reportError("\nRobot configured", false);
        } catch (Exception e) {
            DriverStation.reportError("\nRobot - Error configuring Robot", false);
        }
//        Initialize Modules
        modules = ModuleControl.getInstance().getModules();
        robotControl.set(0, true);
        robotControl.enablePWM(0, .25);
        robotControl.setPWMRate(0, .1);
        DriverStation.reportError("\nRobot Intialized", false);
    }

    /**
     * Runs the motors with tank steering.
     */
    public void operatorControl() {

        while (isOperatorControl() && isEnabled()) {
            //Update Inputs
            sensorInputControl.update();
            driverInputControl.update();
            robotControl.pulse(0,100);
            //Update Module Data
            for(Module m: modules) {
                m.update();
            }
            //Update Module Outputs
            for(Module m: modules) {
                m.updateOutputs();
            }
            Timer.delay(.005);
        }
    }

    public void autonomous() {
        AutonomousRoutine ar = new AutonomousRoutine(this);
        ar.execute();
    }
    
}
