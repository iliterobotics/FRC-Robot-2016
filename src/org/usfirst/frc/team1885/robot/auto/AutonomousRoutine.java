package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.Robot;
import org.usfirst.frc.team1885.robot.common.type.DefenseType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.ActiveIntake;
import org.usfirst.frc.team1885.robot.modules.Shooter;
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
    public static final double CLEAR_SPEED = -1;

    public AutonomousRoutine(Robot r) {
        commands = new LinkedList<AutoCommand>();
        robot = r;
        SensorInputControlSRX.getInstance().calibrateGyro();
        // commands.add(new AutoCalibrateWheels(1));
        DriverStation.reportError("\nGyro Calibrated", false);

//         commands.add(new AutoDriveDistance(3 * 12));
//         commands.add(new AutoAlign(360));
//         commands.add(new AutoWait(2000));
//         commands.add(new AutoAlign());
         getConfiguration();
////         type = DefenseType.MOAT;
         if(!doesNothing) {
             initAutoBreach();
         }
//         if(isHigh){
//             prepareHighGoal();
//         }
//             if(isShooting) {
//                autoMoveToShoot();
//              autoShootBallCam();
//             }
//         }
     }

    public void execute() {
        int commandNum = 0;
        while (!commands.isEmpty() && robot.isEnabled()
                && robot.isAutonomous()) {
            AutoCommand currCommand = commands.peek();
            if (currCommand.isInit()) {
                boolean commandState = currCommand.execute();
                currCommand.updateOutputs();
                if (commandState) {
                    DriverStation.reportError( "\nfinished command " + commandNum, false);
                    commandNum++;
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
//        try{
//        AutonomousConfig autoC = RobotAutonomousConfiguration
//                .pullConfiguration();
//        DriverStation.reportError("\ndefense"  + autoC.getDefense(), false);
//        type = DefenseType.values()[autoC.getDefense()];
//        targetDefense = autoC.getPosition();
//        delay = autoC.getDelay() / 1000.0; // time in seconds
//        isHigh = autoC.getGoalElevation(); // true = high goal, false = low goal
//        goal = autoC.getGoalPosition(); // -1 = Left, 0 = Center, 1 = Right
//        doesNothing = autoC.doesNothing();
//        isShooting = autoC.isShooting();
//
//        DriverStation.reportError(
//                "\n\ndefense#:" + autoC.getDefense() + "defense:" + type
//                        + "\ntargetDefense:" + targetDefense + "\ndelay:"
//                        + delay + "\nisHigh:" + isHigh + "\nGoal:" + goal,
//                false);
//        } catch(Exception e){
            DriverStation.reportError("\nDefense Position" + SensorInputControlSRX.getInstance().getRotaryPosition(), false);
            if(SensorInputControlSRX.getInstance().getRotaryPosition() >= 9){
                doesNothing = true;
            } else{
                type = DefenseType.MOAT;
//            }
        }
    }

    /**
     * Method that initializes all commands for AutonomousRoutine to run
     * CURRENTLY COMMENTED OUT IN ROBOT
     */
    public void initAutoBreach() {
        
        if(type == DefenseType.MOAT || type == DefenseType.RAMPARTS){
            commands.add(new AutoDriveStart(CLEAR_SPEED));
        } else if(type == DefenseType.PORTCULLIS || type == DefenseType.LOW_BAR){
            commands.add(new AutoDriveStart(-START_DRIVE_SPEED));
        }
        else{
            commands.add(new AutoDriveStart(START_DRIVE_SPEED));
        }
        commands.add(new AutoReachedDefense());
        // DEFAULT CASE IS FOR: ROUGH TERRAIN and RAMPARTS
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
        case DRAWBRIDGE:
            autoDrawbridge();
            break;
        case ROCK_WALL:
            autoRockWall();
            break;
        case MOAT:
            autoMoat();
            break;
        default:
            break;
        }
        commands.add(new AutoCrossedDefense());
        commands.add(new AutoAlign());
    }
    
    public void prepareHighGoal(){
        commands.add(new AutoAdjustIntake(ActiveIntake.intakeDown));
        commands.add(new AutoWait(1000));
        commands.add(new AutoShooterTilt(Shooter.HIGH_GOAL_INTAKE_TILT));
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
        if(type != DefenseType.PORTCULLIS && type != DefenseType.MOAT && type != DefenseType.LOW_BAR && type != DefenseType.RAMPARTS) {
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
        commands.add(new AutoAlign(goalTurn));
        /*
        if (!isHigh) {
            commands.add(new AutoDriveStart(START_DRIVE_SPEED));
            commands.add(new AutoCrossedDefense());
            autoShootBall(Shooter.LOW_GOAL_ANGLE);
        }
        else {
            autoShootBall(Shooter.HIGH_GOAL_ANGLE);
            //commands.add(new AutoAimShooter());
        }
        */
        autoShootBallCam();
        }

    /**
     * Controls processes for passing the low bar
     */
    public void autoLowBar() {
        double lowBarTravelDistance = 4.5 * 12; // subject to change from
        ActiveIntake.getInstance().setIntakeSolenoid(ActiveIntake.intakeDown);
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
        commands.add(new AutoDriveStart(-START_DRIVE_SPEED));
    }

    /**
     * Controls processes required for locating the high and low goal and
     * shooting
     * 
     * @param true
     *            = high goal; false = low goal
     */
    public void autoShootBall(double angle) {
        commands.add(new AutoShooterTilt(angle));
        commands.add(new AutoShoot());
        //TODO vision to twist for more accurate
        
    }
    
    public void autoShootBallCam(){
        commands.add(new AutoShooterTilt(Shooter.HIGH_GOAL_ANGLE));
//        commands.add(new AutoShooterInitiate());
        commands.add(new AutoAimShooter());
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

    private final double X_ERROR = 1;
    private final double Y_OVERSHOOT_DISTANCE = 2;

    public void autoSally() {
        // commands.add(); //Check to see if touching sallyport
        double disXInit = 10;
        double disYInit = 23;
        commands.add(new AutoUtilityArm(-disXInit, disYInit));
        double disXHangOver = 20;
        double disYHangOver = 23;
        commands.add(new AutoUtilityArm(-disXHangOver, disYHangOver));
        commands.add(new AutoWait(300));
        double disXGrab = 20;
        double disYGrab = 12;
        commands.add(new AutoUtilityArm(-disXGrab, disYGrab));
        commands.add(new AutoWait(300));
        // double disBack = ;
        // commands.add(); //Drive backwards a certain distance
        // commands.add(new AutoWait(300));
        // commands.add(); //Do a 180
        commands.add(new AutoUtilityArm());
        // commands.add(); //Begin driving forward
    }

    /**
     * Controls process for lowering and crossing the drawbridge
     */
    public void autoDrawbridge() {
        double disXInit = 0;
        double disYInit = 30 + Y_OVERSHOOT_DISTANCE; // Initialize arm position
        commands.add(new AutoUtilityArm(disXInit, disYInit));
        commands.add(new AutoWait(100));
        double disXHangOver = 6 + X_ERROR;
        double disYHangOver = 28.75 + Y_OVERSHOOT_DISTANCE; // Hang over the
        // drawbridge
        commands.add(new AutoUtilityArm(-disXHangOver, disYHangOver));
        commands.add(new AutoWait(300));
        double disXGrabOnto = disXHangOver;
        double disYGrabOnto = 26.25 + Y_OVERSHOOT_DISTANCE; // 'Grab' onto the
                                                            // bridge
        commands.add(new AutoUtilityArm(-disXGrabOnto, disYGrabOnto));
        commands.add(new AutoWait(100));
        // commands.add(new Auto); drive back SLOWLY
        double disXPushDown = 16 + X_ERROR;
        double disYPushDown = -2.3 - Y_OVERSHOOT_DISTANCE; // Push drawbridge
        // the rest of the way down
        commands.add(new AutoUtilityArm(-disXPushDown, disYPushDown));
        commands.add(new AutoWait(100));
        double disXCompactArm = 9 + X_ERROR;
        double disYCompactArm = -4.2 - Y_OVERSHOOT_DISTANCE; // Bring arm
        // closer to us, to make sure the wheels can go over the bridge
        commands.add(new AutoUtilityArm(-disXCompactArm, disYCompactArm));
        commands.add(new AutoWait(300));
        // commands.add(new Auto); drive forward onto ramp, somewhat slow
        commands.add(new AutoWait(1000));
    }
}
