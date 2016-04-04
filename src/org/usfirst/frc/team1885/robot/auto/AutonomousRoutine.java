package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.Robot;
import org.usfirst.frc.team1885.robot.common.type.DefenseType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.ActiveIntake;
import org.usfirst.frc.team1885.robot.modules.Shooter;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.serverdata.RobotAutonomousConfiguration;

import dataclient.robotdata.autonomous.AutonomousConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousRoutine {
    public static final double PITCH_CHANGE_ON_RAMP = 4.5; // NavX is sideways
    public static final double RAMPART_SPEED_MAX = 0.6;
    public static final double RAMPART_SPEED_MIN = 0.5;
    public static final double START_DRIVE_SPEED = 0.5;

    private DefenseType type;
    private int targetDefense;

    private Robot robot;
    private LinkedList<AutoCommand> commands;
    private double delay = 0.005;
    private boolean isHigh;
    private int goal;
    private boolean doesNothing;
    private boolean isShooting;
    private boolean manualOverride;
    public static final double CLEAR_SPEED = 1;

    public boolean configured;

    public AutonomousRoutine(Robot r) {
        commands = new LinkedList<AutoCommand>();
        robot = r;
        SensorInputControlSRX.getInstance().calibrateGyro();
        // commands.add(new AutoCalibrateWheels(1));
        DriverStation.reportError("\nGyro Calibrated", false);

        // commands.add(new AutoDriveDistance(3 * 12));
        // commands.add(new AutoAlign(360));
        // commands.add(new AutoWait(2000));
        // commands.add(new AutoAlign());
        configured = false;
    }

    public void execute() {
        int commandNum = 0;
        while (!commands.isEmpty() && robot.isEnabled()
                && robot.isAutonomous()) {
            if (!configured) {
                getConfiguration();
                try {
                    getServerConfig();
                } catch (Throwable t) {
                    DriverStation.reportError("\nERROR:: Could not retrieve configuration from server", false);
                }
                if (!doesNothing) {
                    initAutoBreach();
                    if (isShooting) {
//                        autoMoveToShoot();
                        prepareHighGoal();
//                        autoShootWithCam(); does not currently work, rely on static measurements
                        autoShootBallCam();
                    }
                }
                configured = true;
            } else {
                AutoCommand currCommand = commands.peek();
                if (currCommand.isInit()) {
                    boolean commandState = currCommand.execute();
                    currCommand.updateOutputs();
                    if (commandState) {
                        DriverStation.reportError(
                                "\nfinished command " + commandNum, false);
                        commandNum++;
                        commands.poll();
                    }
                } else {
                    currCommand.setInit(currCommand.init());
                }
                Timer.delay(delay);
            }
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
        if ((int) (SensorInputControlSRX.getInstance().getRotaryPosition(SensorType.DEFENSE_SELECTION)) >= 5) { // do nothing case
            doesNothing = true;
            isShooting = false;
            manualOverride = false;
        } else {
            manualOverride = true;
            doesNothing = false;
            isShooting = true;
            goal = 0;
            type = DefenseType.MOAT;
            // type =
            // DefenseType.values()[(int)(SensorInputControlSRX.getInstance().getRotaryPosition())];
            DriverStation.reportError("Running Moat with Manual Override", false);
        }
        try {
            if (!manualOverride) {
                getServerConfig();
            }
        } catch (Throwable t) {
            DriverStation.reportError("\nFailed to retrieve config from server",
                    false);
            doesNothing = true;
        }
    }

    public void getServerConfig() {
        AutonomousConfig autoC = RobotAutonomousConfiguration
                .pullConfiguration();
        if (autoC != null) {
            DriverStation.reportError("\ndefense" + autoC.getDefense(), false);
            type = DefenseType.values()[autoC.getDefense()];
            targetDefense = autoC.getPosition();
            delay = autoC.getDelay() / 1000.0; // time in seconds
            isHigh = autoC.getGoalElevation(); // true = high goal, false = low
                                               // goal
            goal = autoC.getGoalPosition(); // -1 = Left, 0 = Center, 1 = Right
            doesNothing = autoC.doesNothing();
            isShooting = autoC.isShooting();

            DriverStation.reportError(
                    "\n\ndefense#:" + autoC.getDefense() + "defense:" + type
                            + "\ntargetDefense:" + targetDefense + "\ndelay:"
                            + delay + "\nisHigh:" + isHigh + "\nGoal:" + goal,
                    false);
        }
    }

    /**
     * Method that initializes all commands for AutonomousRoutine to run
     * CURRENTLY COMMENTED OUT IN ROBOT
     */
    public void initAutoBreach() {
        ActiveIntake.getInstance().setIntakeSolenoid(ActiveIntake.intakeUp); // intake should always start up
        if (type == DefenseType.MOAT || type == DefenseType.RAMPARTS) {
            commands.add(new AutoDriveStart(CLEAR_SPEED));
        } else if (type == DefenseType.PORTCULLIS) {
            commands.add(new AutoDriveStart(-START_DRIVE_SPEED));
        } else {
            commands.add(new AutoDriveStart(START_DRIVE_SPEED));
        }
        commands.add(new AutoReachedDefense());
        // DEFAULT CASE calls autoMoat which is sufficient for moat, ramparts,
        // rough terrain, rock wall
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
        case RAMPARTS:
            DrivetrainControl.getInstance().setLowGear();
        case ROCK_WALL:
            DrivetrainControl.getInstance().setLowGear();
        default:
            autoMoat();
            break;
        }
        commands.add(new AutoCrossedDefense());
        commands.add(new AutoAlign());
    }

    public void prepareHighGoal() {
        commands.add(new AutoAdjustIntake(ActiveIntake.intakeDown));
        commands.add(new AutoWait(1000));
        commands.add(new AutoShooterTilt(Shooter.STATIC_TILT_LIMIT_UPPER));
    }

    public void autoShootWithCam() {
        prepareHighGoal();
        commands.add(new AutoFindHighGoal());
        commands.add(new AutoLineUpHighGoal());
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
            align = 180 + 58 - 360;
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
            align = -180;
        } else if (goal == 1) { // Right Goal
            switch (targetDefense) {
            // DON'T DO 1!
            case 1:
                firstTurn = 90;
                secondMove = 17 * 12;
                break; // Not done yet needs more commands
            case 2:
                firstTurn = 57;
                secondMove = 15.5 * 12;
                break;
            case 3:
                firstTurn = 45;
                secondMove = 12.5 * 12;
                break;
            case 4:
                firstTurn = 30;
                secondMove = 10 * 12;
                break;
            case 5:
                firstTurn = 0;
                secondMove = 9 * 12;
                break;
            default:
                DriverStation.reportError("Invalid Target Defense", false);
            }
            align = 180 - 58;
        } else {
            DriverStation.reportError("Invalid Goal Number", false);
        }
        if (type != DefenseType.PORTCULLIS) {
            firstMove = -firstMove;
            secondMove = -secondMove;
            align += 180;
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
        commands.add(new AutoDriveDistance(firstMove));
        commands.add(new AutoAlign(firstTurn));
        commands.add(new AutoDriveDistance(secondMove));
        commands.add(new AutoAimTurn(goalTurn));
        prepareHighGoal();
        /*
         * if (!isHigh) { commands.add(new AutoDriveStart(START_DRIVE_SPEED));
         * commands.add(new AutoCrossedDefense());
         * autoShootBall(Shooter.LOW_GOAL_ANGLE); } else {
         * autoShootBall(Shooter.HIGH_GOAL_ANGLE); //commands.add(new
         * AutoAimShooter()); }
         */
    }

    /**
     * Controls processes for passing the low bar
     */
    public void autoLowBar() {
        double lowBarTravelDistance = 4.5 * 12; // subject to change from
        ActiveIntake.getInstance().setIntakeSolenoid(ActiveIntake.intakeDown);
        DrivetrainControl.getInstance().setLowGear();
        commands.add(new AutoDriveDistance(lowBarTravelDistance, .2));
    }

    public void autoPortcullis() {
        commands.add(new AutoPortcullis());
    }

    /**
     * Controls processes for crossing the ramparts
     */
    public void autoRamparts() {
        commands.add(new AutoRamparts());
    }

    public void autoCheval() {

    }

    public void autoRockWall() {
        commands.add(new AutoRockWall());
    }

    public void autoMoat() {
        commands.add(new AutoWait(1000));
        commands.add(new AutoDriveStart(START_DRIVE_SPEED));
    }

    /**
     * Controls processes required for locating the high and low goal and
     * shooting
     * 
     * @param angle
     *            angle to position the shooter tilt
     */
    public void autoShootBall(double angle) {
        commands.add(new AutoShooterTilt(angle));
        commands.add(new AutoShoot());

    }

    /**
     * Controls processes for shooting in the high goal using camera vision
     */
    public void autoShootBallCam() {
        commands.add(new AutoShooterAim());
        commands.add(new AutoShoot());
    }

    public void autoWheelie() {
        commands.add(new AutoDriveStart(-1));
        commands.add(new AutoWait(500));
        commands.add(new AutoDriveStart(1));
        commands.add(new AutoWait(300));
        commands.add(new AutoDriveStart(-1));
        commands.add(new AutoCrossedDefense());
    }
}
