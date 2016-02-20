package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.Robot;
import org.usfirst.frc.team1885.robot.common.type.DefenseType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.serverdata.RobotAutonomousConfiguration;

import dataclient.robotdata.autonomous.AutonomousConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousRoutine {
    public static final double PITCH_CHANGE_ON_RAMP = 4.5; // NavX is sideways
    public static final double RAMPART_SPEED_MAX = 0.6;
    public static final double RAMPART_SPEED_MIN = 0.5;
    public static final double START_DRIVE_SPEED = -0.5;

    private DefenseType type;
    private int targetDefense;

    private Robot robot;
    private LinkedList<AutoCommand> commands;
    private double delay = 0.05;
    private boolean isHigh;
    private int goal;

    public AutonomousRoutine(Robot r) {
        commands = new LinkedList<AutoCommand>();
        robot = r;
        SensorInputControlSRX.getInstance().calibrateGyro();
        DriverStation.reportError("Gyro Calibrated", false);
        Timer.delay(3);
        
        getConfiguration();
        initAutoBreach();
        autoMoveToShoot();

        // autoMoveToShoot(needs values)

        // Not finished yet
        // getConfiguration();
        // initAutoBreach(type);
        // autoMoveToShoot();
    }
    public void execute() {
        while (!commands.isEmpty() && robot.isEnabled()
                && robot.isAutonomous()) {
            AutoCommand currCommand = commands.peek();
            if (currCommand.isInit()) {
                boolean commandState = currCommand.execute();
                currCommand.updateOutputs();
                if (commandState) {
                    // DriverStation.reportError(
                    // "\nfinished command " + commands.size(), false);
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

    public void getConfiguration() {
        AutonomousConfig autoC = RobotAutonomousConfiguration
                .pullConfiguration();
        type = DefenseType.values()[autoC.getDefense()];
        targetDefense = autoC.getPosition();
        delay = autoC.getDelay() / 1000.0; // time in seconds
        isHigh = autoC.getGoalElevation(); // true = high goal, false = low goal
        goal = autoC.getGoalPosition(); // -1 = Left, 0 = Center, 1 = Right

        DriverStation.reportError(
                "\n\ndefense#:" + autoC.getDefense() + "defense:" + type
                        + "\ntargetDefense:" + targetDefense + "\ndelay:"
                        + delay + "\nisHigh:" + isHigh + "\nGoal:" + goal,
                false);
    }

    /**
     * Method that initializes all commands for AutonomousRoutine to run
     * CURRENTLY COMMENTED OUT IN ROBOT
     */
    public void initAutoBreach() {
        commands.add(new AutoDriveStart(START_DRIVE_SPEED));
        commands.add(new AutoReachedDefense());

        // DEFAULT CASE IS FOR: MOAT, ROUGH TERRAIN, ROCK WALL
        switch (type) {
        case LOW_BAR:
            autoLowBar();
            break;
        case PORTCULLIS:
            autoPortcullis();
            break;
        case CHEVAL_DE_FRISE:
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
        default:
            break;
        }
        commands.add(new AutoCrossedDefense());
        autoAlign();

    }

    public void initAutoShoot() {
        // TODO Checks for center, left, or right and chooses the right switch
        // statement to run for
        // switch statement based off of defense going for
        // calls individual method inputting values for goal and position
        // method adds the commands for getting to the correct position
    }

    public void autoMoveToShoot() {
        // Huge Switch statement that finds all the parameters for
        // autoMoveToShoot that adds commands
        double firstTurn = 0;
        double firstMove = 0;
        double secondMove = 0;
        double align = 0;
        if (goal == -1) { // Left Goal
            switch (targetDefense) {
            case 1:
                firstTurn = 0;
                secondMove = 6 * 12;
                break;
            case 2:
                firstTurn = -25;
                secondMove = 8.321 * 12;
                break;
            case 3:
                firstTurn = -54.7;
                secondMove = 10 * 12;
                break;
            case 4:
                firstTurn = -64.35;
                secondMove = 13.86 * 12;
                break;
            case 5:
                firstTurn = -71.07;
                secondMove = 18.5 * 12;
                break;
            default:
                DriverStation.reportError("Invalid Target Defense", false);
            }
            align = 58;
        } else if (goal == 0) { // Center Goal
            switch (targetDefense) {
            case 1:
                firstTurn = 90;
                secondMove = 11 * 12;
                break;
            case 2:
                firstTurn = 90;
                secondMove = 7 * 12;
                break;
            case 3:
                firstTurn = 90;
                secondMove = 3 * 12;
                break;
            case 4:
                firstTurn = -90;
                secondMove = 1 * 12;
                break;
            case 5:
                firstTurn = -90;
                secondMove = 3 * 12;
                break;
            default:
                DriverStation.reportError("Invalid Target Defense", false);
            }
            align = 0;
        } else if (goal == 1) { // Right Goal
            switch (targetDefense) {
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            default:
                DriverStation.reportError("Invalid Target Defense", false);
            }
            align = -58;
        } else {
            DriverStation.reportError("Invalid Goal Number", false);
        }
        autoMoveToShoot(firstMove, firstTurn, secondMove, align);
    }

    /**
     * @param firstMove
     *            distance in inches for moving after breaching
     * @param firstTurn
     *            yaw value to turn to to aim towards goal shooting point
     * @param secondMove
     *            distance in inches to move to goal shooting point
     * @param goalTurn
     *            yaw value to turn to to aim at goal
     */
    public void autoMoveToShoot(double firstMove, double firstTurn,
            double secondMove, double goalTurn) {
        commands.add(new AutoDriveDistance(firstMove, true, -.5, -.5));
        commands.add(new AutoAlign(firstTurn));
        commands.add(new AutoDriveDistance(secondMove, true, -.5, -.5));
        commands.add(new AutoAlign(goalTurn));
        if (isHigh) {
            // TODO aim at high goal
            // shoot

        } else {
            // TODO go forward to low goal
            // raise up shooter
            // shoot
        }

    }

    /**
     * Controls processes for passing the low bar
     */
    public void autoLowBar() {
        double lowBarTravelDistance = 4.2 * 12; // subject to change from
                                                // testing
        commands.add(
                new AutoDriveDistance(lowBarTravelDistance, false, -.3, -.3));
    }

    /**
     * Controls processes for crossing the ramparts
     */
    public void autoRamparts() {
        commands.add(new AutoRamparts());
        autoAlign();
    }

    public void autoPortcullis() {

    }

    public void autoCheval() {

    }

    public void autoSally() {

    }

    /**
     * Controls process for lowering and crossing the drawbridge
     */
    public void autoDrawbridge() {
        // Needs to include moving to Drawbridge
        // commands.add(new AutoDrawbridge());
    }
    /**
     * Reusable method to align robot after crossing a defense
     */
    public void autoAlign() {
        commands.add(new AutoAlign());
        commands.add(new AutoWait(5000));
        autoShootHighGoal();
    }
    /**
     * Controls processes required for locating the high goal and shooting
     */
    public void autoShootHighGoal() {
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
