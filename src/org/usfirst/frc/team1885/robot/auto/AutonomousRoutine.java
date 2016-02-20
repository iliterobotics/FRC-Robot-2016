package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.Robot;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.ActiveIntake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousRoutine {
    public static final double PITCH_CHANGE_ON_RAMP = 4.5;
    public static final double RAMPART_SPEED_MAX = 0.6;
    public static final double RAMPART_SPEED_MIN = 0.5;
    public static final double START_DRIVE_SPEED = -0.45;
    public static final double MOAT_CLEAR_SPEED = -0.75;

    private Robot robot;
    private LinkedList<AutoCommand> commands;
    private static final double delay = 0.05;

    public AutonomousRoutine(Robot r) {
        commands = new LinkedList<AutoCommand>();
        robot = r;
        SensorInputControlSRX.getInstance().calibrateGyro();
        DriverStation.reportError("Gyro Calibrated", false);
        Timer.delay(1);
        initAuto();
    }

    public void execute() {
        while (!commands.isEmpty() && robot.isEnabled()
                && robot.isAutonomous()) {
            AutoCommand currCommand = commands.peek();
            if (currCommand.isInit()) {
                boolean commandState = currCommand.execute();
                currCommand.updateOutputs();
                if (commandState) {
                    System.out.println("finished command " + commands.size());
                    commands.poll();
                }
            } else {
                currCommand.setInit(currCommand.init());
            }
            Timer.delay(delay);
        }
    }
    // STANDARD CONFIGURATION
    // AutoStartDrive - begins movement
    // AutoReachedDefense - checks if we have hit the defense (not necessary in
    // all cases)
    // in between checks to cross the defense
    // AutoCrossedDefense - checks if we have landed and can prepare to shoot
    // AutoAlign - realigns the robot to move in position to shoot

    /**
     * Method that initializes all commands for AutonomousRoutine to run
     * CURRENTLY COMMENTED OUT IN ROBOT
     */
    public void initAuto() {
        commands.add(new AutoDriveStart(START_DRIVE_SPEED));
        commands.add(new AutoReachedDefense());
        DefenseType type = DefenseType.LOWBAR; // to be changed to equal the
                                               // analog input
        // DEFAULT CASE IS FOR: MOAT, ROUGH TERRAIN, ROCK WALL
        switch (type) {
        case LOWBAR:
            autoLowBar();
            break;
        case PORTCULLIS:
            autoPortcullis();
            break;
        case CHEVAL:
            autoCheval();
            break;
        case SALLYPORT:
            autoSally();
            break;
        case RAMPARTS:
            autoRamparts();
            break;
        case DRAWBRIDGE:
            autoDrawbridge();
            break;
        case MOAT:
            autoMoat();
        default:
            break;
        }
        commands.add(new AutoCrossedDefense());
        autoAlign();
    }
    /**
     * Adds speed to the robot to clear the moat
     */
    public void autoMoat(){
        commands.add(new AutoDriveStart(MOAT_CLEAR_SPEED));
    }
    /**
     * Controls processes for passing the low bar
     */
    public void autoLowBar() {
        double lowBarTravelDistance = 4.2 * 12; // Best distance from testing
        ActiveIntake.getInstance().intakeDown();
        commands.add(
                new AutoDriveDistance(lowBarTravelDistance, false, -.3, -.3));

    }

    public void autoRamparts() {
        commands.add(new AutoRamparts());
    }

    public void autoPortcullis() {

    }

    public void autoCheval() {

    }

    public void autoSally() {

    }

    public void autoDrawbridge() {

    }

    /**
     * Reusable method to align robot after crossing a defense
     */
    public void autoAlign() {
        commands.add(new AutoAlign());
        // autoShootBall(false);
    }

    /**
     * Controls processes required for locating the high and low goal and
     * shooting
     * 
     * @param true
     *            = high goal; false = low goal
     */
    public void autoShootBall(boolean goal) {

    }
}
