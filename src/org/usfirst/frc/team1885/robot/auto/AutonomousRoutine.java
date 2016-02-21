package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.Robot;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;

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
                            "finished command " + commands.size(), false);
                    commands.poll();
                }
            } else {
                currCommand.setInit(currCommand.init());
            }
            Timer.delay(delay);
        }
    }

    private final double X_ERROR = 1;
    private final double Y_OVERSHOOT_DISTANCE = 0;

    public void drawbridge() {
        double disX1 = 6 + X_ERROR;
        double disY1 = 28.75 + Y_OVERSHOOT_DISTANCE;
        commands.add(new AutoUtilityArm(-disX1, disY1));

        commands.add(new AutoWait(2000));
        commands.add(new AutoUtilityArm());
    }
}
