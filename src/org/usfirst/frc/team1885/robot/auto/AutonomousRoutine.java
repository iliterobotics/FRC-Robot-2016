package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.Robot;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousRoutine {
    public static final double PITCH_CHANGE_ON_RAMP = 4.5;
    public static final double RAMPART_SPEED_MAX = 0.6;
    public static final double RAMPART_SPEED_MIN = 0.5;
    public static final double START_DRIVE_SPEED = 0.5;

    private Robot robot;
    private LinkedList<AutoCommand> commands;
    private static final double delay = 0.05;

    public AutonomousRoutine(Robot r) {
        commands = new LinkedList<AutoCommand>();
        robot = r;
        SensorInputControlSRX.getInstance().calibrateGyro();
        DriverStation.reportError("Stage 0", false);
        commands.add(new AutoUtilityArm(-10, 10));
        DriverStation.reportError("Stage 1", false);
        commands.add(new AutoWait(5000));
        DriverStation.reportError("Stage 2", false);
        commands.add(new AutoUtilityArm(true));
    }
    public void execute() {
        while (!commands.isEmpty() && robot.isEnabled()
                && robot.isAutonomous()) {
            AutoCommand currCommand = commands.peek();
            if (currCommand.isInit()) {
                boolean commandState = currCommand.execute();
                currCommand.updateOutputs();
                if (commandState) {
                    DriverStation.reportError(
                            "finished command " + commands.size(), false);
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
     * Controls processes for passing the Moat, Rough Terrain, and Rock Wall
     */
    public void autoBasicDefense() {
        commands.add(new AutoDriveStart(START_DRIVE_SPEED, START_DRIVE_SPEED));
        commands.add(new AutoReachedDefense());
        commands.add(new AutoCrossedDefense());
        autoAlign();
    }
    /**
     * Controls processes for passing the low bar
     */
    public void autoLowBar() {
        double lowBarTravelDistance = 10; // subject to change from testing

        commands.add(new AutoDriveStart(START_DRIVE_SPEED, START_DRIVE_SPEED));
        commands.add(new AutoReachedDefense());
        commands.add(new AutoDriveDistance(lowBarTravelDistance));
        commands.add(new AutoCrossedDefense());
        autoAlign();
    }
    /**
     * Controls processes for crossing the ramparts
     */
    public void autoRamparts() {
        commands.add(new AutoDriveStart(START_DRIVE_SPEED, START_DRIVE_SPEED));
        commands.add(new AutoReachedDefense());
        commands.add(new AutoRamparts());
        autoAlign();
    }
    /**
     * Controls process for lowering and crossing the drawbridge
     */
    public void autoDrawbridge() {
        commands.add(new AutoArm(90, 95));
        commands.add(new AutoWait(3000));

        commands.add(new AutoArm(0, 170));// Resets the arm to default position
    }
    /**
     * Reusable method to align robot after crossing a defense
     */
    public void autoAlign() {
        commands.add(new AutoAlign());
        autoShootHighGoal();
    }
    /**
     * Controls processes required for locating the high goal and shooting
     */
    public void autoShootHighGoal() {
    }
}
