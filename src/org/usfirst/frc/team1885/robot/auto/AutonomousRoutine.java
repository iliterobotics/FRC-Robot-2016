package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.Robot;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.manipulator.UtilityArm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousRoutine {
    private Robot robot;
    private LinkedList<AutoCommand> commands, drawbridgeCommands,
            sallyPortCommands, chivalCommands;
    private static final double delay = 0.005;

    public AutonomousRoutine(Robot r) {
        commands = new LinkedList<AutoCommand>();
        drawbridgeCommands = new LinkedList<AutoCommand>();
        sallyPortCommands = new LinkedList<AutoCommand>();
        chivalCommands = new LinkedList<AutoCommand>();
        robot = r;
        fillDrawbridgeCommands();
        fillSallyPortCommands();
        fillChivalCommands();

        sallyPort();
        commands.add(new AutoWait(2000));
        sallyPort();
    }
    public void execute() {
        int size = commands.size();
        while (!commands.isEmpty() && robot.isEnabled()
                && robot.isAutonomous()) {
            AutoCommand currCommand = commands.peek();
            if (currCommand.isInit()) {
                boolean commandState = currCommand.execute();
                currCommand.updateOutputs();
                if (commandState) {
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

    public void sallyPort() {
        // commands.add(); //Check to see if touching sallyport
        commands.add(sallyPortCommands.get(0));
        commands.add(new AutoWait(300));
        commands.add(sallyPortCommands.get(1));
        commands.add(new AutoWait(300));
        commands.add(sallyPortCommands.get(2));
        commands.add(new AutoWait(300));
        // double disBack = ;
        // commands.add(); //Drive backwards a certain distance
        // commands.add(new AutoWait(300));
        // commands.add(); //Do a 180
        commands.add(sallyPortCommands.get(3));
        // commands.add(); //Begin driving forward
    }

    public void drawbridge() {
        commands.add(drawbridgeCommands.get(0));
        commands.add(new AutoWait(100));
        commands.add(drawbridgeCommands.get(1));
        commands.add(new AutoWait(300));
        commands.add(drawbridgeCommands.get(2));
        commands.add(new AutoWait(100));
        commands.add(drawbridgeCommands.get(3));
        commands.add(new AutoWait(100));
        commands.add(drawbridgeCommands.get(4));
        commands.add(new AutoWait(300));
        // commands.add(new Auto); drive forward onto ramp, somewhat slow
        commands.add(new AutoWait(1000));
    }
    public void fillDrawbridgeCommands() {
        double disXInit = 0;
        double disYInit = 30 + Y_OVERSHOOT_DISTANCE; // Initialize arm position
        drawbridgeCommands.add(new AutoUtilityArm(-disXInit, disYInit));
        double disXHangOver = 6 + X_ERROR;
        double disYHangOver = 28.75 + Y_OVERSHOOT_DISTANCE; // Hang over the
        // drawbridge
        drawbridgeCommands.add(new AutoUtilityArm(-disXHangOver, disYHangOver));
        double disXGrabOnto = disXHangOver;
        double disYGrabOnto = 26.25 + Y_OVERSHOOT_DISTANCE; // 'Grab' onto the
                                                            // bridge
        drawbridgeCommands.add(new AutoUtilityArm(-disXGrabOnto, disYGrabOnto));
        double disXPushDown = 16 + X_ERROR;
        double disYPushDown = -2.3 - Y_OVERSHOOT_DISTANCE; // Push drawbridge
        // the rest of the way down
        drawbridgeCommands.add(new AutoUtilityArm(-disXPushDown, disYPushDown));
        double disXCompactArm = 9 + X_ERROR;
        double disYCompactArm = -4.2 - Y_OVERSHOOT_DISTANCE; // Bring arm
        // closer to us, to make sure the wheels can go over the bridge
        drawbridgeCommands
                .add(new AutoUtilityArm(-disXCompactArm, disYCompactArm));
    }
    public void fillSallyPortCommands() {
        double disXInit = 10;
        double disYInit = 23;
        sallyPortCommands.add(new AutoUtilityArm(-disXInit, disYInit));
        double disXHangOver = 20;
        double disYHangOver = 23;
        sallyPortCommands.add(new AutoUtilityArm(-disXHangOver, disYHangOver));
        double disXGrab = 20;
        double disYGrab = 12;
        sallyPortCommands.add(new AutoUtilityArm(-disXGrab, disYGrab));
        sallyPortCommands.add(new AutoUtilityArm());
    }
    public void fillChivalCommands() {

    }
}
