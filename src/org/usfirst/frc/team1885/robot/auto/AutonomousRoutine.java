package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.Robot;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousRoutine {
    private Robot robot;
    private LinkedList<AutoCommand> commands;
    private static final double delay = 0.05;

    public AutonomousRoutine(Robot r) {
        commands = new LinkedList<AutoCommand>();
        robot = r;
        commands.add(new AutoUtilityArm(-10, 10));

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
    public void testArm() {
        double jointAPosition = 90.0;
        double jointBPosition = (180.0 - 9.866);
        commands.add(new AutoArm(.25, .50, jointAPosition, jointBPosition));
        jointBPosition -= 62;
        commands.add(new AutoArm(.25, -.25, jointAPosition, jointBPosition));
    }
}
