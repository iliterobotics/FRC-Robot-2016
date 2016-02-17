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
    private SensorInputControlSRX sensorSRX = SensorInputControlSRX
            .getInstance();

    public AutonomousRoutine(Robot r) {
        commands = new LinkedList<AutoCommand>();
        robot = r;
        SensorInputControlSRX.getInstance().calibrateGyro();
        DriverStation.reportError("Gyro Calibrated", false);
        Timer.delay(3);
        commands.add(new AutoDriveDistance(1 * 12, true));
        // commands.add(new AutoTurnEnc(90, 10));
    }
    public void execute() {
        while (!commands.isEmpty() && robot.isEnabled()
                && robot.isAutonomous()) {
            sensorSRX.update();
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
        commands.add(new AutoDriveDistance(lowBarTravelDistance, false));
        commands.add(new AutoCrossedDefense());
        autoAlign();
    }
    public void autoRamparts() {
        commands.add(new AutoDriveStart(START_DRIVE_SPEED, START_DRIVE_SPEED));
        commands.add(new AutoReachedDefense());
        commands.add(new AutoRamparts());
        commands.add(new AutoCrossedDefense());
        autoAlign();
    }
    /**
     * Reusable method to align robot after crossing a defense
     */
    public void autoAlign() {
        commands.add(new AutoAlign());
        autoShootBall(false);
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
