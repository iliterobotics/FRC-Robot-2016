package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousRoutine {
    private Robot robot;
    private LinkedList<AutoCommand> commands;
    private static final double delay = 0.005;

    public AutonomousRoutine(Robot r) {
        commands = new LinkedList<AutoCommand>();
        robot = r;
        drawbridge();
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
                            "\nfinished command " + commands.size(), false);
                    commands.poll();
                }
            } else {
                currCommand.setInit(currCommand.init());
            }
            Timer.delay(delay);
        }
    }

    private final double X_ERROR = 1;
    private final double Y_OVERSHOOT_DISTANCE = 2;

    public void drawbridge() {
        double disXInit = 0;
        double disYInit = 30 + Y_OVERSHOOT_DISTANCE;
        commands.add(new AutoUtilityArm(disXInit, disYInit));
        commands.add(new AutoWait(500));
        double disXHangOver = 6 + X_ERROR;
        double disYHangOver = 28.75 + Y_OVERSHOOT_DISTANCE;
        commands.add(new AutoUtilityArm(-disXHangOver, disYHangOver));
        commands.add(new AutoWait(500));
        double disXGrabOnto = disXHangOver;
        double disYGrabOnto = 26.25 + Y_OVERSHOOT_DISTANCE;
        commands.add(new AutoUtilityArm(-disXGrabOnto, disYGrabOnto));
        commands.add(new AutoWait(500));
        // commands.add(new Auto); drive back SLOWLY
        double disXPushDown = 16 + X_ERROR;
        double disYPushDown = -2.3 - Y_OVERSHOOT_DISTANCE;
        commands.add(new AutoUtilityArm(-disXPushDown, disYPushDown));
        commands.add(new AutoWait(500));
        double disXCompactArm = 9 + X_ERROR;
        double disYCompactArm = -4.2 - Y_OVERSHOOT_DISTANCE;
        commands.add(new AutoUtilityArm(-disXCompactArm, disYCompactArm));
        commands.add(new AutoWait(500));
        // commands.add(new Auto); drive forward onto ramp, somewhat slow
        commands.add(new AutoWait(1000));
        commands.add(new AutoUtilityArm());
    }
}
